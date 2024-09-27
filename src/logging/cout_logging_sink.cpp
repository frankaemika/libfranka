// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <iostream>

#include <franka/logging/cout_logging_sink.hpp>

const std::string kCoutLoggingSinkName = "CoutLoggingSink";

namespace franka {

auto CoutLoggingSink::getName() const -> std::string {
  return kCoutLoggingSinkName;
}

auto CoutLoggingSink::logInfo(const std::string& message) -> void {
  std::cout << "INFO: " << message << std::endl;
}

auto CoutLoggingSink::logWarn(const std::string& message) -> void {
  std::cout << "WARNING: " << message << std::endl;
}

auto CoutLoggingSink::logError(const std::string& message) -> void {
  std::cout << "ERROR: " << message << std::endl;
}

}  // namespace franka
