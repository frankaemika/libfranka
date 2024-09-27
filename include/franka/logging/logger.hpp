// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <fmt/format.h>
#include <memory>

#include <franka/logging/logging_sink_interface.hpp>

/**
 * Provides logging functionality. A user can implement a custom logger by inheriting from the
 * LoggingSinkInterface and adding it the stored loggers.
 * Now, whenever libfranka wants to log anything, it will call the logInfo, logWarn, or logError
 * methods of all stored loggers.
 */
namespace franka::logging {

/**
 * Adds a logger to the list of loggers.
 * @param logger The logger to add.
 */
auto addLogger(const std::shared_ptr<LoggingSinkInterface>& logger) -> void;

/**
 * Removes a logger from the list of loggers.
 * @param logger_name The name of the logger to remove.
 */
auto removeLogger(const std::string& logger_name) -> void;

/**
 * Removes all loggers from the list of loggers.
 */
auto removeAllLoggers() -> void;

/**
 * Logs an info message.
 * @param message The message to log.
 */
auto logInfo(const std::string& message) -> void;

/**
 * Logs an info message.
 * @tparam S The type of the format string.
 * @tparam Args The types of the arguments.
 * @param format_str The format string.
 * @param args The arguments.
 */
template <typename S, typename... Args>
auto logInfo(const S& format_str, Args&&... args) -> void {
  logInfo(fmt::format(format_str, std::forward<Args>(args)...));
}

/**
 * Logs a warning message.
 * @param message The message to log.
 */
auto logWarn(const std::string& message) -> void;

/**
 * Logs a warning message.
 * @tparam S The type of the format string.
 * @tparam Args The types of the arguments.
 * @param format_str The format string.
 * @param args The arguments.
 */
template <typename S, typename... Args>
auto logWarn(const S& format_str, Args&&... args) -> void {
  logWarn(fmt::format(format_str, std::forward<Args>(args)...));
}

/**
 * Logs an error message.
 * @param message The message to log.
 */
auto logError(const std::string& message) -> void;

/**
 * Logs an error message.
 * @tparam S The type of the format string.
 * @tparam Args The types of the arguments.
 * @param format_str The format string.
 * @param args The arguments.
 */
template <typename S, typename... Args>
auto logError(const S& format_str, Args&&... args) -> void {
  logError(fmt::format(format_str, std::forward<Args>(args)...));
}

}  // namespace franka::logging
