#include "network.h"

#include <franka/robot.h>

namespace franka {

void readBytes(Poco::Net::StreamSocket& socket,
               void* buffer,
               int length,
               std::chrono::seconds timeoutInSeconds) {
  using std::chrono::steady_clock;
  using std::chrono::duration_cast;

  int bytes_read = 0;
  auto start_time = steady_clock::now();
  while (bytes_read < length) {
    bytes_read +=
        socket.receiveBytes((char*)buffer + bytes_read, length - bytes_read, 0);
    auto elapsed_time = steady_clock::now() - start_time;
    if (elapsed_time > timeoutInSeconds) {
      throw franka::NetworkException("libfranka: stream socket read timeout.");
    }
  }
}

}  // namespace franka