#pragma once

#include <franka/robot_state.h>

#include "robot_control.h"

class MockRobotControl : public franka::RobotControl {
 public:
  MOCK_METHOD0(startController, void());
  MOCK_METHOD0(stopController, void());

  MOCK_METHOD4(startMotion,
               void(research_interface::Move::ControllerMode controller_mode,
                    research_interface::Move::MotionGeneratorMode motion_generator_mode,
                    const research_interface::Move::Deviation& maximum_path_deviation,
                    const research_interface::Move::Deviation& maximum_goal_pose_deviation));
  MOCK_METHOD0(stopMotion, void());

  MOCK_METHOD1(update, franka::RobotState(const research_interface::ControllerCommand& command));
  MOCK_METHOD2(update,
               franka::RobotState(const research_interface::MotionGeneratorCommand& motion_command,
                                  const research_interface::ControllerCommand& control_command));

  franka::RealtimeConfig realtimeConfig() const noexcept override {
    return franka::RealtimeConfig::kIgnore;
  }
};
