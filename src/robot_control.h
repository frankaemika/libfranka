#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

namespace franka {

class RobotControl {
 public:
  virtual ~RobotControl() = default;

  virtual void startController() = 0;
  virtual void stopController() = 0;

  virtual void startMotion(
      research_interface::robot::Move::ControllerMode controller_mode,
      research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::robot::Move::Deviation& maximum_path_deviation,
      const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) = 0;
  virtual void stopMotion() = 0;

  virtual RobotState update(
      const research_interface::robot::MotionGeneratorCommand* motion_command = nullptr,
      const research_interface::robot::ControllerCommand* control_command = nullptr) = 0;

  virtual RealtimeConfig realtimeConfig() const noexcept = 0;
};

}  // namespace franka
