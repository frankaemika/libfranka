// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <iostream>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>

#include <tracy/Tracy.hpp>

#include <franka/exception.h>

#define PACKET_HW_TIMESTAMPS

namespace franka {

class Network {
 public:
  Network(const std::string& franka_address,
          uint16_t franka_port,
          std::chrono::milliseconds tcp_timeout = std::chrono::seconds(60),
          std::chrono::milliseconds udp_timeout = std::chrono::seconds(1),
          std::tuple<bool, int, int, int> tcp_keepalive = std::make_tuple(true, 1, 3, 1));
  ~Network();

  uint16_t udpPort() const noexcept;

  template <typename T>
  T udpBlockingReceive();

  template <typename T>
  bool udpReceive(T* data);

  template <typename T>
  void udpSend(const T& data);

  void tcpThrowIfConnectionClosed();

  /**
   * Blocks until a T::Response message with the given command ID has been received.
   *
   * Additional variable-length data for the expected response (if any) is written into the given
   * vl_buffer. If vl_buffer is not given, this data is discarded.
   *
   * @param[in] command_id Expected command ID of the T::Response.
   * @param[out] vl_buffer If given, variable-length data for the expected T::Response message (if
   * any has been received) is copied into it.
   *
   * @return Received T::Response instance.
   */
  template <typename T>
  typename T::Response tcpBlockingReceiveResponse(uint32_t command_id,
                                                  std::vector<uint8_t>* vl_buffer = nullptr);

  /**
   * Tries to receive a T::Response message with the given command ID (non-blocking).
   *
   * Additional variable-length data for the expected response (if any) is discarded.
   *
   * @param[in] command_id Expected command ID of the T::Response.
   * @param[in] handler Callback to be invoked if the expected response has been received.
   *
   * @return True if a T::Response message with the given command_id has been received, false
   * otherwise.
   */
  template <typename T>
  bool tcpReceiveResponse(uint32_t command_id,
                          std::function<void(const typename T::Response&)> handler);

  template <typename T, typename... TArgs>
  uint32_t tcpSendRequest(TArgs&&... args);

 private:
  template <typename T>
  T udpBlockingReceiveUnsafe();

  template <typename T>
  void tcpReadFromBuffer(std::chrono::microseconds timeout);

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;
  uint16_t udp_port_;

  std::mutex tcp_mutex_;
  std::mutex udp_mutex_;

  uint32_t command_id_{0};

  std::vector<uint8_t> pending_response_{};
  size_t pending_response_offset_ = 0;
  uint32_t pending_command_id_ = 0;
  std::unordered_map<uint32_t, std::vector<uint8_t>> received_responses_{};
};

template <typename T>
bool Network::udpReceive(T* data) {
  std::lock_guard<std::mutex> _(udp_mutex_);

  if (udp_socket_.available() >= static_cast<int>(sizeof(T))) {
    *data = udpBlockingReceiveUnsafe<T>();
    return true;
  }
  return false;
}

static void make_address(unsigned short port,
                         struct sockaddr_in *host_address) {
  bzero(host_address, sizeof(struct sockaddr_in));

  host_address->sin_family = AF_INET;
  host_address->sin_port = htons(port);
  host_address->sin_addr.s_addr = INADDR_ANY;
}

struct packet_time_t {
  uint64_t nic_ts;
  uint64_t kernel_ts;
  uint64_t user_ts;
};

inline packet_time_t parse_time(struct timespec *ts) {
  /* Hardware timestamping provides three timestamps -
   *   system (software)
   *   transformed (hw converted to sw)
   *   raw (hardware)
   * in that order - though depending on socket option, you may have 0 in
   * some of them.
   */
  // printf("timestamps " TIME_FMT TIME_FMT TIME_FMT "\n",
  //   (uint64_t)ts[0].tv_sec, (uint64_t)ts[0].tv_nsec,
  //   (uint64_t)ts[1].tv_sec, (uint64_t)ts[1].tv_nsec,
  //   (uint64_t)ts[2].tv_sec, (uint64_t)ts[2].tv_nsec );

  if (ts == nullptr) {
    return packet_time_t();
  }

  auto sys_now = std::chrono::system_clock::now();
  uint64_t nanoseconds_user = std::chrono::duration_cast<std::chrono::nanoseconds>(sys_now.time_since_epoch()).count();
  // printf("time_user : %d.%06d\n", (int) time_user.tv_sec,
  //                                 (int) time_user.tv_usec);

  uint64_t nanoseconds_nic = ts[2].tv_sec * 1000000000 + ts[2].tv_nsec;
  uint64_t nanoseconds_kernel = ts[0].tv_sec * 1000000000 + ts[0].tv_nsec;

  return packet_time_t{
    .nic_ts = nanoseconds_nic,
    .kernel_ts = nanoseconds_kernel,
    .user_ts = nanoseconds_user
  };
}

inline packet_time_t handle_time(struct msghdr *msg) {
  struct timespec *ts = NULL;
  struct cmsghdr *cmsg;

  for (cmsg = CMSG_FIRSTHDR(msg); cmsg; cmsg = CMSG_NXTHDR(msg, cmsg)) {
    if (cmsg->cmsg_level != SOL_SOCKET)
      continue;

    switch (cmsg->cmsg_type) {
    case SO_TIMESTAMPNS:
      ts = (struct timespec *)CMSG_DATA(cmsg);
      break;
    case SO_TIMESTAMPING:
      ts = (struct timespec *)CMSG_DATA(cmsg);
      break;
    default:
      /* Ignore other cmsg options */
      break;
    }
  }

  return parse_time(ts);
}

template <typename T> int udp_timestamp_receive(std::array<uint8_t, sizeof(T)>& buffer, Poco::Net::DatagramSocket& socket, Poco::Net::SocketAddress& addr) {
  struct msghdr msg;
  struct iovec iov;
  struct sockaddr_in host_address;
  char control[1024];
  int got;

  /* recvmsg header structure */
  make_address(0, &host_address);
  iov.iov_base = buffer.data();
  iov.iov_len = buffer.size();
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_name = &host_address;
  msg.msg_namelen = sizeof(struct sockaddr_in);
  msg.msg_control = control;
  msg.msg_controllen = 1024;

  /* block for message */
  got = recvmsg(socket.impl()->sockfd(), &msg, 0);

  if (got == -1) {
    if (errno == EAGAIN) {
      return 0;
    } else {
      throw NetworkException(strerror(errno));
    }
  }

  addr = Poco::Net::SocketAddress(reinterpret_cast<sockaddr*>(&host_address), sizeof(struct sockaddr_in));

  packet_time_t packet_time = handle_time(&msg);
  //char packet_timing_str[1024];
  //sprintf(packet_timing_str, "nic:%lu,kernel:%lu,user:%lu", packet_time.nic_ts, packet_time.kernel_ts, packet_time.user_ts);

  //TracyMessage(packet_timing_str, strlen(packet_timing_str));

  NICTime::last_nic_time.nic_time_ns = packet_time.nic_ts;
  NICTime::last_nic_time.kernel_time_ns = packet_time.kernel_ts;
  NICTime::last_nic_time.user_time_ns = packet_time.user_ts;

  return got;

}

template <typename T>
T Network::udpBlockingReceive() {
  std::lock_guard<std::mutex> _(udp_mutex_);
  return udpBlockingReceiveUnsafe<T>();
}

template <typename T>
T Network::udpBlockingReceiveUnsafe() try {
  std::array<uint8_t, sizeof(T)> buffer;

#ifdef PACKET_HW_TIMESTAMPS
  int bytes_received = udp_timestamp_receive<T>(buffer, udp_socket_, udp_server_address_);
#else
  int bytes_received =
      udp_socket_.receiveFrom(buffer.data(), static_cast<int>(buffer.size()), udp_server_address_);
#endif

  if (bytes_received != static_cast<int>(buffer.size())) {
    throw ProtocolException("libfranka: incorrect object size");
  }

  return *reinterpret_cast<T*>(buffer.data());
} catch (const Poco::Exception& e) {
    std::cout << "Exception  " << e.what() << std::endl; 
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
  throw NetworkException("libfranka: UDP receive: "s + e.what());
}

template <typename T>
void Network::udpSend(const T& data) try {
  std::lock_guard<std::mutex> _(udp_mutex_);

  int bytes_sent = udp_socket_.sendTo(&data, sizeof(data), udp_server_address_);
  if (bytes_sent != sizeof(data)) {
    throw NetworkException("libfranka: could not send UDP data");
  }
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
  throw NetworkException("libfranka: UDP send: "s + e.what());
}

template <typename T>
void Network::tcpReadFromBuffer(std::chrono::microseconds timeout) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_ERROR)) {
    throw NetworkException("libfranka: TCP connection got interrupted.");
  }

  if (!tcp_socket_.poll(timeout.count(), Poco::Net::Socket::SELECT_READ)) {
    return;
  }

  int available_bytes = tcp_socket_.available();
  if (pending_response_.empty() &&
      available_bytes >= static_cast<int>(sizeof(typename T::Header))) {
    typename T::Header header;
    tcp_socket_.receiveBytes(&header, sizeof(header));
    if (header.size < sizeof(header)) {
      throw ProtocolException("libfranka: Incorrect TCP message size.");
    }
    pending_response_.resize(header.size);
    std::memcpy(pending_response_.data(), &header, sizeof(header));
    pending_response_offset_ = sizeof(header);
    pending_command_id_ = header.command_id;
  }
  if (!pending_response_.empty() && available_bytes > 0) {
    pending_response_offset_ += tcp_socket_.receiveBytes(
        &pending_response_[pending_response_offset_],
        std::min(tcp_socket_.available(),
                 static_cast<int>(pending_response_.size() - pending_response_offset_)));
    if (pending_response_offset_ == pending_response_.size()) {
      received_responses_.emplace(pending_command_id_, pending_response_);
      pending_response_.clear();
      pending_response_offset_ = 0;
      pending_command_id_ = 0;
    }
  }
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
  throw NetworkException("libfranka: TCP receive: "s + e.what());
}

template <typename T, typename... TArgs>
uint32_t Network::tcpSendRequest(TArgs&&... args) try {
  std::lock_guard<std::mutex> _(tcp_mutex_);

  typename T::template Message<typename T::Request> message(
      typename T::Header(T::kCommand, command_id_++,
                         sizeof(typename T::template Message<typename T::Request>)),
      typename T::Request(std::forward<TArgs>(args)...));

  tcp_socket_.sendBytes(&message, sizeof(message));

  return message.header.command_id;
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
  throw NetworkException("libfranka: TCP send bytes: "s + e.what());
}

template <typename T>
bool Network::tcpReceiveResponse(uint32_t command_id,
                                 std::function<void(const typename T::Response&)> handler) {
  using namespace std::literals::chrono_literals;  // NOLINT(google-build-using-namespace)
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }

  tcpReadFromBuffer<T>(0us);
  decltype(received_responses_)::const_iterator it = received_responses_.find(command_id);
  if (it != received_responses_.end()) {
    auto message = reinterpret_cast<const typename T::template Message<typename T::Response>*>(
        it->second.data());
    if (message->header.size < sizeof(message)) {
      throw ProtocolException("libfranka: Incorrect TCP message size.");
    }
    handler(message->getInstance());
    received_responses_.erase(it);
    return true;
  }
  return false;
}

template <typename T>
typename T::Response Network::tcpBlockingReceiveResponse(uint32_t command_id,
                                                         std::vector<uint8_t>* vl_buffer) {
  using namespace std::literals::chrono_literals;  // NOLINT(google-build-using-namespace)
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::defer_lock);
  decltype(received_responses_)::const_iterator it;
  do {
    lock.lock();
    tcpReadFromBuffer<T>(10ms);
    it = received_responses_.find(command_id);
    lock.unlock();
    std::this_thread::yield();
  } while (it == received_responses_.end());

  auto message = *reinterpret_cast<const typename T::template Message<typename T::Response>*>(
      it->second.data());
  if (message.header.size < sizeof(message)) {
    throw ProtocolException("libfranka: Incorrect TCP message size.");
  }

  if (vl_buffer != nullptr && message.header.size != sizeof(message)) {
    size_t data_size = message.header.size - sizeof(message);
    std::vector<uint8_t> data_buffer(data_size);
    std::memcpy(data_buffer.data(), &it->second[sizeof(message)], data_size);
    *vl_buffer = data_buffer;
  }

  received_responses_.erase(it);
  return message.getInstance();
}

template <typename T, uint16_t kLibraryVersion>
void connect(Network& network, uint16_t* ri_version) {
  uint32_t command_id = network.tcpSendRequest<T>(network.udpPort());
  typename T::Response connect_response = network.tcpBlockingReceiveResponse<T>(command_id);
  switch (connect_response.status) {
    case (T::Status::kIncompatibleLibraryVersion):
      throw IncompatibleVersionException(connect_response.version, kLibraryVersion);
    case (T::Status::kSuccess):
      *ri_version = connect_response.version;
      break;
    default:
      throw ProtocolException("libfranka: Protocol error during connection attempt");
  }
}

}  // namespace franka
