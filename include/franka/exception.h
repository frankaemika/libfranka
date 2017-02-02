#pragma once

#include <stdexcept>
#include <string>

/**
 * @file exception.h
 * Contains exception definitions.
 */

namespace franka {

/**
 * Base class for all exceptions used by `libfranka`.
 */
struct Exception : public std::runtime_error {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit Exception(std::string const& message);
};

/**
 * NetworkException is thrown when a connection to FRANKA cannot be established,
 * or when a timeout occurs.
 */
struct NetworkException : public Exception {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit NetworkException(std::string const& message);
};

/**
 * ProtocolException is thrown when the server returns an incorrect message.
 */
struct ProtocolException : public Exception {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit ProtocolException(std::string const& message);
};

/**
 * IncompatibleVersionException is thrown if the server does not support this
 * version of libfranka.
 */
struct IncompatibleVersionException : public Exception {
  /**
   * Constructs the exception.
   *
   * @param[in] message more detailed description of the error
   */
  explicit IncompatibleVersionException(std::string const& message);
};

}  // namespace franka
