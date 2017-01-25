#pragma  once

#include <boost/asio.hpp>
#include <boost/lambda/bind.hpp>

#include <franka/robot.h>

namespace franka
{
/**
 * BlockingReadWrite implements blocking read and write calls. This class does not handle
 * connecting or accepting connections.
 *
 * Based on the following examples:
 * http://www.boost.org/doc/libs/1_52_0/doc/html/boost_asio/example/timeouts/blocking_tcp_client.cpp
 * http://www.boost.org/doc/libs/1_52_0/doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
 */
template <typename Socket>
class BlockingReadWrite {
 public:
  BlockingReadWrite(std::shared_ptr<boost::asio::io_service> service, std::shared_ptr<Socket> socket)
      : io_service_{service},
        socket_{socket}
  {
  }

  void write(const unsigned char *data, size_t size,
             boost::posix_time::time_duration timeout_duration)
  {
    // Asio guarantees that its asynchronous operations will never fail
    // with would_block, so any other value in ec indicates completion.
    boost::system::error_code write_error = boost::asio::error::would_block;

    boost::asio::deadline_timer timer(*io_service_, timeout_duration);
    bool timeout_fired = false;
    timer.async_wait([&] ( const boost::system::error_code& ec ) {
                          if (ec != boost::asio::error::operation_aborted)
                          {
                            io_service_->stop();
                            timeout_fired = true;
                          }
                        });

    std::size_t bytes_written = 0;
    auto buffer = boost::asio::buffer(data, size);
    boost::asio::async_write(*socket_, buffer,
                             [&](const boost::system::error_code& ec, std::size_t length) {
                               timer.cancel();
                               write_error = ec;
                               bytes_written = length;
                             });

    // Block until the asynchronous operation has completed.
    io_service_->run();
    io_service_->reset();

    if(timeout_fired)
    {
      throw franka::NetworkException("libfranka: socket write timeout");
    }else if (write_error)
    {
      throw boost::system::system_error(write_error);
    }
  }

  std::vector<unsigned char> receive(size_t size, boost::posix_time::time_duration timeout)
  {
    // TODO: apply timeout for the whole operation, not just the single receive call
    std::vector<unsigned char> data(size);

    size_t received = 0;
    while(received < size)
    {
      size_t bytes_left = size - received;
      received += receive(data.data() + received, bytes_left, timeout);
    }
    return data;
  }


 private:
  std::size_t receive(unsigned char* data, size_t size,
                      boost::posix_time::time_duration timeout_duration)
  {
    boost::asio::deadline_timer timer(*io_service_, timeout_duration);
    bool timeout_fired = false;
    timer.async_wait([&] ( const boost::system::error_code& ec ) {
      if (ec != boost::asio::error::operation_aborted) {
        io_service_->stop();
        timeout_fired = true;
      }
    });

    boost::system::error_code read_error = boost::asio::error::would_block;
    std::size_t bytes_read = 0;
    boost::asio::async_read(*socket_, boost::asio::buffer(data, size),
                             [&](const boost::system::error_code& ec, std::size_t length) {
                               timer.cancel();
                               read_error = ec;
                               bytes_read = length;
                             });

    // Block until all asynchronous operation were completed.
    io_service_->run();
    io_service_->reset();

    if(timeout_fired)
    {
      throw franka::NetworkException("libfranka: socket read timeout");
    }else if (read_error)
    {
      throw boost::system::system_error(read_error);
    }
    return bytes_read;
  }

 private:
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<Socket> socket_;
};
} // namespace franka


