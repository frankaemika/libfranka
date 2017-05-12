#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/rbk_types.h>
#include <research_interface/service_types.h>

namespace franka {

class RobotControl {
 public:
  virtual void startController() = 0;
  virtual void stopController() = 0;

  virtual void startMotionGenerator(
      research_interface::StartMotionGenerator::MotionGeneratorMode mode) = 0;
  virtual void stopMotionGenerator() = 0;

  virtual bool update() = 0;

  virtual const RobotState& robotState() const noexcept = 0;
  virtual void controllerCommand(const research_interface::ControllerCommand&
                                     controller_command) noexcept = 0;
  virtual void motionGeneratorCommand(
      const research_interface::MotionGeneratorCommand&
          motion_generator_command) noexcept = 0;
  virtual RealtimeConfig realtimeConfig() const noexcept = 0;
};

}  // namespace franka
