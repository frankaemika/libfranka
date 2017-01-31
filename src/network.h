#pragma once

#include <Poco/Net/StreamSocket.h>
#include <franka/robot.h>
#include <chrono>

namespace franka {

template <class T>
void readObject(Poco::Net::StreamSocket& socket,
                T& object,
                std::chrono::seconds timeoutInSeconds) {
  using std::chrono::steady_clock;
  using std::chrono::duration_cast;

  uint8_t* buff = reinterpret_cast<uint8_t*>(&object);  // NOLINT

  int bytes_read = 0;
  constexpr int bytes_total = sizeof(T);

  auto start_time = steady_clock::now();
  while (bytes_read < bytes_total) {
    int bytes_left = bytes_total - bytes_read;
    bytes_read += socket.receiveBytes(buff + bytes_read,  // NOLINT
                                      bytes_left, 0);

    auto elapsed_time = steady_clock::now() - start_time;
    if (elapsed_time > timeoutInSeconds) {
      throw franka::NetworkException("libfranka: stream socket read timeout.");
    }
  }
}

}  // namespace franka
