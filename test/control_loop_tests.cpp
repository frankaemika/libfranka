#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "helpers.h"
#include "mock_robot_control.h"
#include "motion_loop.h"

using namespace ::testing;

using franka::Duration;
using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::robot::ControllerCommand;
using research_interface::robot::Move;
using research_interface::robot::MotionGeneratorCommand;

using std::placeholders::_1;
using std::placeholders::_2;

class ControlLoop : public franka::MotionLoop<franka::Torques> {
 public:
  using franka::MotionLoop<franka::Torques>::MotionLoop;
  using franka::MotionLoop<franka::Torques>::controllerSpinOnce;
};

struct MockControlCallback {
  MOCK_METHOD2(invoke, Torques(const RobotState&, Duration));
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

  EXPECT_NO_THROW(
      ControlLoop(robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2)));
}

TEST(ControlLoop, SpinOnce) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});

  RobotState robot_state{};
  Duration duration(1);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));

  ControlLoop loop(robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2));

  ControllerCommand command;
  EXPECT_TRUE(loop.controllerSpinOnce(robot_state, duration, &command));
  EXPECT_EQ(torques.tau_J, command.tau_J_d);
}

TEST(ControlLoop, SpinWithStoppingCallback) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;

  RobotState robot_state{};
  Duration duration(2);
  Duration zero_duration(0);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(Stop));

  ControlLoop loop(robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2));

  // Use ASSERT to abort on failure because loop() further down
  // would block otherwise
  ASSERT_FALSE(loop.controllerSpinOnce(robot_state, duration, nullptr));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(control_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}

TEST(ControlLoop, GetsCorrectTimeStep) {
  NiceMock<MockRobotControl> robot;

  MockControlCallback control_callback;
  RobotState robot_state{};
  robot_state.time = Duration(10);
  Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(control_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const RobotState&, Duration duration) -> Torques {
        EXPECT_EQ(ticks.at(control_count), duration.ms());

        if (++control_count == ticks.size()) {
          return Stop;
        }
        return zero_torques;
      }));
  size_t robot_count = 0;
  EXPECT_CALL(robot, update(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const MotionGeneratorCommand*, const ControllerCommand*) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  ControlLoop loop(robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2));
  loop();
}
