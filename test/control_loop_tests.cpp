#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "helpers.h"
#include "mock_robot_control.h"
#include "motion_loop.h"

using namespace ::testing;

using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::robot::ControllerCommand;
using research_interface::robot::Move;

class ControlLoop : public franka::MotionLoop<franka::Torques> {
 public:
  using franka::MotionLoop<franka::Torques>::MotionLoop;
  using franka::MotionLoop<franka::Torques>::controllerSpinOnce;
};

struct MockControlCallback {
  MOCK_METHOD1(invoke, Torques(const RobotState&));
};

TEST(ControlLoop, CanNotConstructWithoutCallback) {
  MockRobotControl robot;
  EXPECT_THROW(ControlLoop(robot, ControlLoop::ControlCallback(), nullptr), std::invalid_argument);
}

TEST(ControlLoop, CanConstructWithCallback) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   Move::MotionGeneratorMode::kIdle, ControlLoop::kDefaultDeviation,
                                   ControlLoop::kDefaultDeviation));
    EXPECT_CALL(robot, stopMotion());
  }

  StrictMock<MockControlCallback> control_callback;

  EXPECT_NO_THROW(ControlLoop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1)));
}

TEST(ControlLoop, SpinOnce) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});

  RobotState robot_state{};

  EXPECT_CALL(control_callback, invoke(Ref(robot_state))).WillOnce(Return(torques));

  ControlLoop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1));

  ControllerCommand command;
  EXPECT_TRUE(loop.controllerSpinOnce(robot_state, &command));
  EXPECT_EQ(torques.tau_J, command.tau_J_d);
}

TEST(ControlLoop, SpinWithStoppingCallback) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;

  RobotState robot_state{};
  EXPECT_CALL(control_callback, invoke(Ref(robot_state))).WillOnce(Return(Stop));

  ControlLoop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1));

  // Use ASSERT to abort on failure because loop() further down
  // would block otherwise
  ASSERT_FALSE(loop.controllerSpinOnce(robot_state, nullptr));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(control_callback, invoke(_)).WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}
