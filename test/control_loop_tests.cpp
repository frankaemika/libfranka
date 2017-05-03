#include <functional>

#include <gmock/gmock.h>

#include "control_loop.h"

#include "mock_robot_control.h"

using ::testing::InSequence;
using ::testing::Eq;
using ::testing::Field;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::NiceMock;
using ::testing::_;

using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::ControllerCommand;

class ControlLoop : public franka::ControlLoop {
 public:
  using franka::ControlLoop::ControlLoop;
  using franka::ControlLoop::spinOnce;
};

class MockControlCallback {
 public:
  MOCK_METHOD1(invoke, Torques(const RobotState&));
};

TEST(ControlLoop, CanConstructWithoutCallback) {
  MockRobotControl robot;
  EXPECT_CALL(robot, startController()).Times(0);
  EXPECT_CALL(robot, stopController()).Times(0);

  ControlLoop loop(robot, ControlLoop::ControlCallback());
}

TEST(ControlLoop, CanConstructWithCallback) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startController());
    EXPECT_CALL(robot, stopController());
  }

  ControlLoop loop(robot, [](const franka::RobotState&){
    return Torques({0, 1, 2, 3, 4, 5, 6});
  });
}

TEST(ControlLoop, SpinOnceWithoutCallback) {
  MockRobotControl robot;

  ControlLoop loop(robot, ControlLoop::ControlCallback());
  EXPECT_TRUE(loop.spinOnce());
}

TEST(ControlLoop, SpinOnceWithCallback) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});

  RobotState robot_state;
  EXPECT_CALL(robot, robotStateMock()).WillOnce(ReturnRef(robot_state));
  EXPECT_CALL(robot, controllerCommandMock(Field(&research_interface::ControllerCommand::tau_J_d, Eq(torques.tau_J))));

  EXPECT_CALL(control_callback, invoke(_))
    .WillOnce(Return(torques));

  ControlLoop loop(robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1));
  EXPECT_TRUE(loop.spinOnce());
}

TEST(ControlLoop, SpinOnceWithStoppingCallback) {
  NiceMock<MockRobotControl> robot;
  RobotState robot_state;
  ON_CALL(robot, robotStateMock())
    .WillByDefault(ReturnRef(robot_state));
  ON_CALL(robot, update())
    .WillByDefault(Return(true));

  ControlLoop loop(robot, [](const RobotState&) { return Stop; });

  ASSERT_FALSE(loop.spinOnce());
  loop();
}
