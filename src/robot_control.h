#pragma once

#include <franka/robot_state.h>
#include <franka/control_types.h>
#include <research_interface/rbk_types.h>
#include <research_interface/types.h>

namespace franka {

class RobotControl {
 public:
  virtual ~RobotControl() = default;

  virtual void startController() = 0;
  virtual void stopController() = 0;

  virtual void startMotionGenerator(research_interface::StartMotionGeneratorRequest::Type type) = 0;
  virtual void stopMotionGenerator() = 0;

  virtual bool update() = 0;

  virtual const RobotState& robotState() const noexcept = 0;
  virtual research_interface::ControllerCommand& controllerCommand() noexcept = 0;
  virtual research_interface::MotionGeneratorCommand& motionGeneratorCommand() noexcept = 0;
  virtual RealtimeConfig realtimeConfig() const noexcept = 0;
};

}  // namespace franka
