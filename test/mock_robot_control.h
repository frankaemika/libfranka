#pragma once

#include "robot_control.h"

class MockRobotControl : public franka::RobotControl {
 public:
  MOCK_METHOD0(startController, void());
  MOCK_METHOD0(stopController, void());

  MOCK_METHOD1(startMotionGenerator, void(research_interface::StartMotionGenerator::MotionGeneratorMode));
  MOCK_METHOD0(stopMotionGenerator, void());

  MOCK_METHOD0(update, bool());

  MOCK_CONST_METHOD0(robotStateMock, const franka::RobotState&());
  MOCK_METHOD1(controllerCommandMock, void(const research_interface::ControllerCommand&));
  MOCK_METHOD1(motionGeneratorCommandMock, void(const research_interface::MotionGeneratorCommand&));

  // Workaround because Google Mock does not support noexcept.
  const franka::RobotState& robotState() const noexcept override {
    return robotStateMock();
  }
  void controllerCommand(const research_interface::ControllerCommand& command) noexcept override {
    controllerCommandMock(command);
  }
  void motionGeneratorCommand(const research_interface::MotionGeneratorCommand& command) noexcept override {
    motionGeneratorCommandMock(command);
  }
  franka::RealtimeConfig realtimeConfig() const noexcept override {
    return franka::RealtimeConfig::kIgnore;
  }
};
