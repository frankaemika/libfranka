#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <utility>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>

#include <franka/exception.h>

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

  template <typename T>
  typename T::Response tcpBlockingReceiveResponse(uint32_t command_id,
                                                  std::vector<uint8_t>* buffer = nullptr);

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

  std::unique_ptr<uint8_t[]> pending_response_{};
  size_t pending_response_offset_ = 0;
  std::unordered_map<uint32_t, std::unique_ptr<uint8_t[]>> received_responses_{};
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

template <typename T>
T Network::udpBlockingReceive() {
  std::lock_guard<std::mutex> _(udp_mutex_);
  return udpBlockingReceiveUnsafe<T>();
}

template <typename T>
T Network::udpBlockingReceiveUnsafe() try {
  std::array<uint8_t, sizeof(T)> buffer;

  int bytes_received =
      udp_socket_.receiveFrom(buffer.data(), static_cast<int>(buffer.size()), udp_server_address_);

  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka: incorrect object size");
  }

  return *reinterpret_cast<T*>(buffer.data());
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
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
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: UDP send: "s + e.what());
}

template <typename T>
void Network::tcpReadFromBuffer(std::chrono::microseconds timeout) try {
  if (!tcp_socket_.poll(timeout.count(), Poco::Net::Socket::SELECT_READ)) {
    return;
  }

  int available_bytes = tcp_socket_.available();
  if (!pending_response_ && available_bytes >= static_cast<int>(sizeof(typename T::Header))) {
    typename T::Header header;
    tcp_socket_.receiveBytes(&header, sizeof(header));
    pending_response_.reset(new uint8_t[header.size]);
    std::memcpy(pending_response_.get(), &header, sizeof(header));
    pending_response_offset_ = sizeof(header);
  }
  if (pending_response_ && available_bytes > 0) {
    typename T::Header* header = reinterpret_cast<typename T::Header*>(pending_response_.get());
    pending_response_offset_ += tcp_socket_.receiveBytes(
        &pending_response_[pending_response_offset_],
        std::min(tcp_socket_.available(),
                 static_cast<int>(header->size - pending_response_offset_)));
    if (pending_response_offset_ == header->size) {
      received_responses_.emplace(header->command_id, std::move(pending_response_));
      pending_response_offset_ = 0;
    }
  }
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: TCP receive: "s + e.what());
}

template <typename T, typename... TArgs>
uint32_t Network::tcpSendRequest(TArgs&&... args) try {
  std::lock_guard<std::mutex> _(tcp_mutex_);

  typename T::template Message<typename T::Request> message(
      typename T::Header(T::kCommand, command_id_++, sizeof(message)),
      typename T::Request(std::forward<TArgs>(args)...));

  tcp_socket_.sendBytes(&message, sizeof(message));

  return message.header.command_id;
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: TCP send bytes: "s + e.what());
}

template <typename T>
bool Network::tcpReceiveResponse(uint32_t command_id,
                                 std::function<void(const typename T::Response&)> handler) {
  using namespace std::literals::chrono_literals;  // NOLINT (google-build-using-namespace)
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }

  tcpReadFromBuffer<T>(0us);
  decltype(received_responses_)::const_iterator it = received_responses_.find(command_id);
  if (it != received_responses_.end()) {
    auto message =
        reinterpret_cast<typename T::template Message<typename T::Response>*>(it->second.get());
    handler(message->getInstance());
    received_responses_.erase(it);
    return true;
  }
  return false;
}

template <typename T>
typename T::Response Network::tcpBlockingReceiveResponse(uint32_t command_id,
                                                         std::vector<uint8_t>* buffer) {
  using namespace std::literals::chrono_literals;  // NOLINT (google-build-using-namespace)
  std::unique_lock<std::mutex> lock(tcp_mutex_, std::defer_lock);
  decltype(received_responses_)::const_iterator it;
  do {
    lock.lock();
    tcpReadFromBuffer<T>(10'000us);
    it = received_responses_.find(command_id);
    lock.unlock();
  } while (it == received_responses_.end());

  auto message =
      *reinterpret_cast<typename T::template Message<typename T::Response>*>(it->second.get());

  if (buffer != nullptr && message.header.size != sizeof(message)) {
    size_t data_size = message.header.size - sizeof(message);
    std::vector<uint8_t> data_buffer(data_size);
    std::memcpy(data_buffer.data(), &it->second[sizeof(message)], data_size);
    *buffer = data_buffer;
  }

  received_responses_.erase(it);
  return message.getInstance();
}

template <typename T, uint16_t kLibraryVersion>
void connect(Network& network, uint16_t* ri_version) {
  uint32_t command_id = network.tcpSendRequest<T>(network.udpPort());
  typename T::Response connect_response = network.tcpBlockingReceiveResponse<T>(command_id);
  switch (connect_response.status) {
    case (T::Status::kIncompatibleLibraryVersion): {
      std::stringstream message;
      message << "libfranka: incompatible library version. " << std::endl
              << "Server version: " << connect_response.version << std::endl
              << "Library version: " << kLibraryVersion;
      throw IncompatibleVersionException(message.str());
    }
    case (T::Status::kSuccess): {
      *ri_version = connect_response.version;
      break;
    }
    default:
      throw ProtocolException(
          "libfranka gripper: protocol error during gripper connection attempt");
  }
}

}  // namespace franka
