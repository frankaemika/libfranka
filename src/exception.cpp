#include <franka/exception.h>

namespace franka {

Exception::Exception(std::string const& message)
    : std::runtime_error(message) {}

NetworkException::NetworkException(std::string const& message)
    : Exception(message) {}

ProtocolException::ProtocolException(std::string const& message)
    : Exception(message) {}

IncompatibleVersionException::IncompatibleVersionException(
    std::string const& message)
    : Exception(message) {}

}  // namespace franka
