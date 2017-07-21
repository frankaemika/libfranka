#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <sstream>
#include <unordered_set>

#include <franka/exception.h>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds tcp_timeout = std::chrono::seconds(60),
                   std::chrono::milliseconds udp_timeout = std::chrono::seconds(1),
                   std::tuple<bool, int, int, int> tcp_keepalive = std::make_tuple(true, 1, 3, 1));
  ~Network();

  template <typename T>
  T udpRead();
  int udpAvailableData();

  uint16_t udpPort() const noexcept;

  void checkTcpConnection();

  template <typename T>
  typename T::Response tcpBlockingReceiveResponse();

  template <typename T>
  void udpSend(const T& data);

  template <typename T>
  void tcpSendRequest(const typename T::Request& request);

  template <typename T>
  bool tcpReadResponse(T* function);

  template <typename T>
  bool tcpHandleResponse(std::function<void(const typename T::Response&)> handler);

  void tcpReceiveIntoBuffer(uint8_t* buffer, size_t read_size);

 private:
  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;

  std::vector<uint8_t> read_buffer_;
};

template <typename T>
T Network::udpRead() try {
  std::array<uint8_t, sizeof(T)> buffer;
  do {
    int bytes_received = udp_socket_.receiveFrom(buffer.data(), static_cast<int>(buffer.size()),
                                                 udp_server_address_);

    if (bytes_received != buffer.size()) {
      throw ProtocolException("libfranka: incorrect object size");
    }
  } while (udp_socket_.available() > 0);

  return *reinterpret_cast<T*>(buffer.data());
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: UDP read: "s + e.what());
}

template <typename T>
void Network::udpSend(const T& data) try {
  int bytes_sent = udp_socket_.sendTo(&data, sizeof(data), udp_server_address_);
  if (bytes_sent != sizeof(data)) {
    throw NetworkException("libfranka: could not send UDP data");
  }
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: UDP send: "s + e.what());
}

template <typename T>
void Network::tcpSendRequest(const typename T::Request& request) try {
  tcp_socket_.sendBytes(&request, sizeof(request));
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: TCP send bytes: "s + e.what());
}

template <typename T>
bool Network::tcpReadResponse(T* function) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    size_t offset = read_buffer_.size();
    int bytes_available = tcp_socket_.available();
    read_buffer_.resize(offset + bytes_available);
    tcpReceiveIntoBuffer(&read_buffer_[offset], bytes_available);

    if (read_buffer_.size() < sizeof(T)) {
      return false;
    }

    *function = *reinterpret_cast<T*>(read_buffer_.data());
    return true;
  }
  return false;
} catch (const Poco::Exception& e) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)
  throw NetworkException("libfranka: "s + e.what());
}

template <typename T>
bool Network::tcpHandleResponse(std::function<void(const typename T::Response&)> handler) {
  if (read_buffer_.size() < sizeof(typename T::Response)) {
    return false;
  }

  typename T::Response response = *reinterpret_cast<typename T::Response*>(read_buffer_.data());

  size_t remaining_bytes = read_buffer_.size() - sizeof(response);
  std::memmove(read_buffer_.data(), &read_buffer_[sizeof(response)], remaining_bytes);
  read_buffer_.resize(remaining_bytes);

  handler(response);
  return true;
}

template <typename T>
typename T::Response Network::tcpBlockingReceiveResponse() {
  std::array<uint8_t, sizeof(typename T::Response)> buffer;
  tcpReceiveIntoBuffer(buffer.data(), buffer.size());
  typename T::Response const* response =
      reinterpret_cast<const typename T::Response*>(buffer.data());
  if (response->function != T::kFunction) {
    throw ProtocolException("libfranka: received response of wrong type");
  }
  return *response;
}

template <typename T, uint16_t kLibraryVersion>
void connect(Network& network, uint16_t* ri_version) {
  typename T::Request connect_request(network.udpPort());
  network.tcpSendRequest<T>(connect_request);

  typename T::Response connect_response = network.tcpBlockingReceiveResponse<T>();
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
