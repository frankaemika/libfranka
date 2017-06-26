#pragma once

#include <chrono>
#include <cstring>
#include <functional>
#include <unordered_set>

#include <franka/exception.h>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/NetException.h>
#include <Poco/Net/StreamSocket.h>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

class Network {
 public:
  explicit Network(const std::string& franka_address,
                   uint16_t franka_port,
                   std::chrono::milliseconds timeout);
  virtual ~Network();

  template <typename T>
  T udpRead();

  virtual uint16_t udpPort() const noexcept;

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

 protected:
  int tcpReceiveIntoBuffer();

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
  Poco::Net::SocketAddress udp_server_address_;

  std::vector<uint8_t> read_buffer_;
};

template <typename T>
T Network::udpRead() try {
  std::array<uint8_t, sizeof(T)> buffer;
  int bytes_received =
      udp_socket_.receiveFrom(buffer.data(), static_cast<int>(buffer.size()), udp_server_address_);
  if (bytes_received != buffer.size()) {
    throw ProtocolException("libfranka: incorrect object size");
  }
  return *reinterpret_cast<T*>(buffer.data());
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: UDP read: "s + e.what());
}

template <typename T>
void Network::udpSend(const T& data) try {
  int bytes_sent = udp_socket_.sendTo(&data, sizeof(data), udp_server_address_);
  if (bytes_sent != sizeof(data)) {
    throw NetworkException("libfranka: UDP - could not send data");
  }
} catch (const Poco::Exception& e) {
  throw NetworkException("libfranka: UDP send: "s + e.what());
}

template <typename T>
void Network::tcpSendRequest(const typename T::Request& request) try {
  tcp_socket_.sendBytes(&request, sizeof(request));
} catch (const Poco::Exception& e) {
  throw NetworkException(std::string{"libfranka: tcp send bytes: "} + e.what());
}

template <typename T>
bool Network::tcpReadResponse(T* function) try {
  if (tcp_socket_.poll(0, Poco::Net::Socket::SELECT_READ)) {
    int rv = tcpReceiveIntoBuffer();

    if (rv == 0) {
      throw NetworkException("libfranka: server closed connection");
    }

    if (read_buffer_.size() < sizeof(T)) {
      return false;
    }

    *function = *reinterpret_cast<T*>(read_buffer_.data());
    return true;
  }
  return false;
} catch (const Poco::Exception& e) {
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
  int bytes_read = 0;
  constexpr int kBytesTotal = sizeof(typename T::Response);
  std::array<uint8_t, kBytesTotal> buffer;

  try {
    while (bytes_read < kBytesTotal) {
      int bytes_left = kBytesTotal - bytes_read;
      int rv = tcp_socket_.receiveBytes(&buffer.at(bytes_read), bytes_left);
      bytes_read += rv;
    }
  } catch (const Poco::TimeoutException& e) {
    if (bytes_read != 0) {
      throw ProtocolException(std::string{"libfranka: incorrect object size"});
    } else {
      throw NetworkException(std::string{"libfranka: connection timeout"});
    }
  } catch (const Poco::Exception& e) {
    throw NetworkException(std::string{"libfranka: connection closed"});
  }
  typename T::Response const* response =
      reinterpret_cast<const typename T::Response*>(buffer.data());
  if (response->function != T::kFunction) {
    throw ProtocolException(std::string{"libfranka: received response of wrong type"});
  }
  return *response;
}

}  // namespace franka
