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

  virtual void startMotion(
      research_interface::Move::ControllerMode controller_mode,
      research_interface::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::Move::Deviation& maximum_path_deviation,
      const research_interface::Move::Deviation& maximum_goal_pose_deviation) = 0;
  virtual void stopMotion() = 0;

  virtual const research_interface::RobotState& update(
      const research_interface::ControllerCommand& command) = 0;
  virtual const research_interface::RobotState& update(
      const research_interface::MotionGeneratorCommand& motion_command,
      const research_interface::ControllerCommand& control_command) = 0;

  virtual RealtimeConfig realtimeConfig() const noexcept = 0;
};

}  // namespace franka
