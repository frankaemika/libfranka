// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <string>

namespace franka {

/**
 * Interface for a logging sink. Implement this interface to create a custom logger.
 */
class LoggingSinkInterface {
 public:
  virtual ~LoggingSinkInterface() = default;

  virtual auto getName() const -> std::string = 0;

  /**
   * Logs an info message.
   * @param message The message to log.
   */
  virtual auto logInfo(const std::string& message) -> void = 0;

  /**
   * Logs a warning message.
   * @param message The message to log.
   */
  virtual auto logWarn(const std::string& message) -> void = 0;

  /**
   * Logs an error message.
   * @param message The message to log.
   */
  virtual auto logError(const std::string& message) -> void = 0;
};

}  // namespace franka
