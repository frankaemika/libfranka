// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * This example demonstrates how to create a custom logger for libfranka which is using the custom
 * logger to forward messages. Those messages could be logged to a file, a database, or any other
 * intended tool.
 *
 * Note: A pre-defined std::cout logger already exists in libfranka, see
 * franka/logging/cout_logging_sink.hpp.
 */

#include <iostream>

#include <franka/logging/logger.hpp>
#include <franka/logging/logging_sink_interface.hpp>

class ExampleLogger : public franka::LoggingSinkInterface {
 public:
  ~ExampleLogger() override = default;

  auto getName() const -> std::string override { return name_; }

  auto logInfo(const std::string& message) -> void override {
    std::cout << "INFO: " << message << std::endl;
  }

  auto logWarn(const std::string& message) -> void override {
    std::cout << "WARNING: " << message << std::endl;
  }

  auto logError(const std::string& message) -> void override {
    std::cout << "ERROR: " << message << std::endl;
  }

 private:
  std::string name_ = "ExampleLogger";
};

int main() {
  auto example_logger = std::make_shared<ExampleLogger>();
  franka::logging::addLogger(example_logger);

  // libfranka will internally use these log lines to forward messages to the custom logger.
  franka::logging::logInfo("This is an info message.");
  franka::logging::logWarn("This is a warning message.");
  franka::logging::logError("This is an error message.");

  return 0;
}
