#pragma once

#include <chrono>

#include <franka/robot.h>
#include <research_interface/rbk_types.h>
#include <research_interface/types.h>

#include "command_handler.h"
#include "complete_robot_state.h"
#include "network/network.h"
#include "network/poco_socket.h"
#include "robot_control.h"

namespace franka {

class Robot::Impl : public RobotControl, public CommandHandler {
 public:
  static constexpr std::chrono::seconds kDefaultTimeout{5};
  static constexpr double kCommandTimeStep{0.001};

  explicit Impl(const std::string& franka_address,
                uint16_t franka_port = research_interface::kCommandPort,
                std::chrono::milliseconds timeout = kDefaultTimeout,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  void setRobotState(const research_interface::RobotState& robot_state);
  bool update() override;

  void controllerCommand(const research_interface::ControllerCommand&
                             controller_command) noexcept override;
  void motionGeneratorCommand(const research_interface::MotionGeneratorCommand&
                                  motion_generator_command) noexcept override;
  const RobotState& robotState() const noexcept override;
  ServerVersion serverVersion() const noexcept;
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  void startMotionGenerator(
      research_interface::StartMotionGeneratorRequest::Type
          motion_generator_type) override;
  void stopMotionGenerator() override;

  template <research_interface::StartMotionGeneratorRequest::Type, typename T>
  void control(
      std::function<T(const RobotState&)> control_callback,
      std::function<void(const T&, research_interface::MotionGeneratorCommand*)>
          conversion_callback);

  template <typename T>
  void control(
      std::function<T(const RobotState&)> control_callback,
      std::function<void(const T&, research_interface::ControllerCommand*)>
          conversion_callback);

  void handleStartMotionGeneratorReply(
      const research_interface::StartMotionGeneratorReply& reply) override;
  void handleStopMotionGeneratorReply(
      const research_interface::StopMotionGeneratorReply& reply) override;
  void handleStartControllerReply(
      const research_interface::StartControllerReply& reply) override;
  void handleStopControllerReply(
      const research_interface::StopControllerReply& reply) override;

 private:
  const RealtimeConfig realtime_config_;

  Network network_;

  bool motion_generator_running_;
  bool controller_running_;
  research_interface::RobotCommand robot_command_;
  CompleteRobotState robot_state_;
};

}  // namespace franka
