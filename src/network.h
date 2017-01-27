#pragma once

#include <Poco/Net/StreamSocket.h>
#include <chrono>

namespace franka {

void readBytes(Poco::Net::StreamSocket& socket,
               void* buffer,
               int length,
               std::chrono::seconds timeoutInSeconds);

}  // namespace franka
