#pragma once

#include "robot_control.h"

class MockRobotControl : public franka::RobotControl {
 public:
  MOCK_METHOD0(startController, void());
  MOCK_METHOD0(stopController, void());

  MOCK_METHOD1(startMotionGenerator, void(research_interface::StartMotionGeneratorRequest::Type));
  MOCK_METHOD0(stopMotionGenerator, void());

  MOCK_METHOD0(update, bool());

  MOCK_CONST_METHOD0(robotStateMock, const franka::RobotState&());
  MOCK_METHOD0(controllerCommandMock, research_interface::ControllerCommand&());
  MOCK_METHOD0(motionGeneratorCommandMock, research_interface::MotionGeneratorCommand&());

  // Workaround because Google Mock does not support noexcept.
  const franka::RobotState& robotState() const noexcept override {
    return robotStateMock();
  }
  research_interface::ControllerCommand& controllerCommand() noexcept override {
    return controllerCommandMock();
  }
  research_interface::MotionGeneratorCommand& motionGeneratorCommand() noexcept override {
    return motionGeneratorCommandMock();
  }
  franka::RealtimeConfig realtimeConfig() const noexcept override {
    return franka::RealtimeConfig::kIgnore;
  }
};
