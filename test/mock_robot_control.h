#pragma once

#include <franka/robot_state.h>

#include "robot_control.h"

class MockRobotControl : public franka::RobotControl {
 public:
  MOCK_METHOD0(startController, void());
  MOCK_METHOD0(stopController, void());

  MOCK_METHOD4(
      startMotion,
      uint32_t(research_interface::robot::Move::ControllerMode controller_mode,
               research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
               const research_interface::robot::Move::Deviation& maximum_path_deviation,
               const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation));
  MOCK_METHOD1(stopMotion, void(uint32_t motion_id));

  MOCK_METHOD2(
      update,
      franka::RobotState(const research_interface::robot::MotionGeneratorCommand* motion_command,
                         const research_interface::robot::ControllerCommand* control_command));

  MOCK_METHOD2(throwOnMotionError,
               void(const franka::RobotState& robot_state, const uint32_t* motion_id));

  franka::RealtimeConfig realtimeConfig() const noexcept override {
    return franka::RealtimeConfig::kIgnore;
  }
};
