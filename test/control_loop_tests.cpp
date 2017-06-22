#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "control_loop.h"

#include "helpers.h"
#include "mock_robot_control.h"

using namespace ::testing;

using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::robot::ControllerCommand;

class ControlLoop : public franka::ControlLoop {
 public:
  using franka::ControlLoop::ControlLoop;
  using franka::ControlLoopBase::spinOnce;
};

struct MockControlCallback {
  MOCK_METHOD1(invoke, Torques(const RobotState&));
};

TEST(ControlLoop, CanNotConstructWithoutCallback) {
  MockRobotControl robot;
  EXPECT_THROW(ControlLoop(robot, ControlLoop::ControlCallback()), std::invalid_argument);
}

TEST(ControlLoop, CanConstructWithCallback) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startController());
    EXPECT_CALL(robot, stopController());
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
  EXPECT_TRUE(loop.spinOnce(robot_state, &command));
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
  ASSERT_FALSE(loop.spinOnce(robot_state, nullptr));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(control_callback, invoke(_)).WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}
