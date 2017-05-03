#pragma once

#include <chrono>

#include <franka/exception.h>

namespace franka {

struct TimeoutException : public Exception {
  using Exception::Exception;
};

class Socket {
 public:
  virtual ~Socket(){};

  virtual void setReceiveTimeout(std::chrono::milliseconds timeout) = 0;
};

class TcpSocket : public Socket {
 public:
  virtual void connect(const std::string& franka_address,
               uint16_t franka_port,
               std::chrono::milliseconds timeout) = 0;
  virtual void setBlocking(bool flag) = 0;
  virtual void setSendTimeout(std::chrono::milliseconds timeout) = 0;
  virtual bool poll() = 0;
  virtual int available() = 0;
  virtual int receiveBytes(void* data, size_t size) = 0;
  virtual int sendBytes(const void *data, size_t size) = 0;
};

class UdpSocket : public Socket {
 public:
  virtual void bind(const std::string& franka_address,
                    uint16_t franka_port) = 0;
  virtual int sendTo(const void *data, size_t size) = 0;
  virtual int receiveFrom(void* data, size_t size) = 0;
  virtual uint16_t port() = 0;
};

} // namespace franka