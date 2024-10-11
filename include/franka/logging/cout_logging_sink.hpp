// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/logging/logging_sink_interface.hpp>

namespace franka {

/**
 * A example logging sink that logs messages to the console.
 */
class CoutLoggingSink : public LoggingSinkInterface {
 public:
  virtual ~CoutLoggingSink() = default;

  // Inherited via LoggingSinkInterface
  auto getName() const -> std::string override;
  auto logInfo(const std::string& message) -> void override;
  auto logWarn(const std::string& message) -> void override;
  auto logError(const std::string& message) -> void override;
};

}  // namespace franka
