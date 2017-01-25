#pragma once

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/lambda/bind.hpp>
#include <chrono>

#include <franka/robot.h>

namespace franka {

template <typename Socket>
void blockingWrite(boost::asio::io_service& io_service,
                   Socket& socket,
                   const unsigned char* data,
                   size_t size,
                   std::chrono::seconds timeout_duration) {
  // Asio guarantees that its asynchronous operations will never fail
  // with would_block, so any other value in ec indicates completion.
  boost::system::error_code write_error = boost::asio::error::would_block;

  boost::asio::steady_timer timer(io_service, timeout_duration);
  bool timeout_fired = false;
  timer.async_wait([&](const boost::system::error_code& ec) {
    if (ec != boost::asio::error::operation_aborted) {
      io_service.stop();
      timeout_fired = true;
    }
  });

  std::size_t bytes_written = 0;
  auto buffer = boost::asio::buffer(data, size);
  boost::asio::async_write(
      socket, buffer,
      [&](const boost::system::error_code& ec, std::size_t length) {
        timer.cancel();
        write_error = ec;
        bytes_written = length;
      });

  // Block until the asynchronous operation has completed.
  io_service.run();
  io_service.reset();

  if (timeout_fired) {
    throw franka::NetworkException("libfranka: socket write timeout");
  } else if (write_error) {
    throw boost::system::system_error(write_error);
  }
}

template <typename Socket>
std::size_t blockingRead(boost::asio::io_service& io_service,
                         Socket& socket,
                         unsigned char* data,
                         size_t size,
                         std::chrono::seconds timeout_duration) {
  boost::asio::steady_timer timer(io_service, timeout_duration);
  bool timeout_fired = false;
  timer.async_wait([&](const boost::system::error_code& ec) {
    if (ec != boost::asio::error::operation_aborted) {
      io_service.stop();
      timeout_fired = true;
    }
  });

  boost::system::error_code read_error = boost::asio::error::would_block;
  std::size_t bytes_read = 0;
  boost::asio::async_read(
      socket, boost::asio::buffer(data, size),
      [&](const boost::system::error_code& ec, std::size_t length) {
        timer.cancel();
        read_error = ec;
        bytes_read = length;
      });

  // Block until all asynchronous operation were completed.
  io_service.run();
  io_service.reset();

  if (timeout_fired) {
    throw franka::NetworkException("libfranka: socket read timeout");
  } else if (read_error) {
    throw boost::system::system_error(read_error);
  }
  return bytes_read;
}

template <typename Socket>
std::vector<unsigned char> blockingReadBytes(
    boost::asio::io_service& io_service,
    Socket& socket,
    size_t size,
    std::chrono::seconds timeout) {
  auto time_start = std::chrono::system_clock::now();
  std::vector<unsigned char> data(size);

  size_t received = 0;
  while (received < size) {
    size_t bytes_left = size - received;
    received += blockingRead(io_service, socket, data.data() + received,
                             bytes_left, timeout);

    auto time_elapsed = std::chrono::system_clock::now() - time_start;
    if (time_elapsed > timeout) {
      if (received > 0) {
        throw franka::ProtocolException("libfranka: invalid object");
      } else {
        throw franka::NetworkException("libfranka: socket read timeout");
      }
    }
  }
  return data;
}

}  // namespace franka
