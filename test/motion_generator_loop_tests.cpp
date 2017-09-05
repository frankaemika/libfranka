#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "motion_generator_loop.h"
#include "motion_generator_traits.h"

#include "helpers.h"
#include "mock_robot_control.h"

using namespace ::testing;

using franka::CartesianPose;
using franka::CartesianVelocities;
using franka::ControllerMode;
using franka::Duration;
using franka::JointPositions;
using franka::JointVelocities;
using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::robot::ControllerCommand;
using research_interface::robot::MotionGeneratorCommand;
using research_interface::robot::Move;
using research_interface::robot::RobotCommand;

using std::placeholders::_1;
using std::placeholders::_2;

class MockControlCallback {
 public:
  MOCK_METHOD2(invoke, Torques(const RobotState&, Duration));
};

template <typename T>
class MotionGeneratorLoop : public franka::MotionGeneratorLoop<T> {
 public:
  using franka::MotionGeneratorLoop<T>::MotionGeneratorLoop;
  using franka::MotionGeneratorLoop<T>::spinOnce;
};

template <typename T>
struct MockMotionCallback {
  MOCK_METHOD2_T(invoke, T(const RobotState&, Duration));
};

template <typename T>
class MotionGeneratorLoops : public ::testing::Test {
 public:
  using Loop = MotionGeneratorLoop<T>;
  using TMotion = T;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::robot::Move::MotionGeneratorMode kMotionGeneratorMode =
      franka::MotionGeneratorTraits<T>::kMotionGeneratorMode;

  T createMotion();
  auto getField(const T& values);
};

template <>
JointPositions MotionGeneratorLoops<JointPositions>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto MotionGeneratorLoops<JointPositions>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_d, Eq(values.q));
}

template <>
JointVelocities MotionGeneratorLoops<JointVelocities>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto MotionGeneratorLoops<JointVelocities>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_d, Eq(velocities.dq));
}

template <>
CartesianPose MotionGeneratorLoops<CartesianPose>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
auto MotionGeneratorLoops<CartesianPose>::getField(const CartesianPose& pose) {
  return Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_d, Eq(pose.O_T_EE));
}

template <>
CartesianVelocities MotionGeneratorLoops<CartesianVelocities>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
auto MotionGeneratorLoops<CartesianVelocities>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_d,
               Eq(cartesian_velocities.O_dP_EE));
}

using MotionTypes =
    ::testing::Types<JointPositions, JointVelocities, CartesianPose, CartesianVelocities>;
TYPED_TEST_CASE(MotionGeneratorLoops, MotionTypes);

TYPED_TEST(MotionGeneratorLoops, CanNotConstructWithoutMotionCallback) {
  StrictMock<MockRobotControl> robot;

  EXPECT_THROW(typename TestFixture::Loop loop(robot,
                                               [](const RobotState&, Duration) {
                                                 return Torques({0, 1, 2, 3, 4, 5, 6});
                                               },
                                               typename TestFixture::MotionGeneratorCallback()),
               std::invalid_argument);
  EXPECT_THROW(typename TestFixture::Loop loop(robot, ControllerMode::kCartesianImpedance,
                                               typename TestFixture::MotionGeneratorCallback()),
               std::invalid_argument);
}

TYPED_TEST(MotionGeneratorLoops, CanNotConstructWithoutControlCallback) {
  StrictMock<MockRobotControl> robot;

  EXPECT_THROW(typename TestFixture::Loop loop(robot, typename TestFixture::ControlCallback(),
                                               std::bind(&TestFixture::createMotion, this)),
               std::invalid_argument);
}

TYPED_TEST(MotionGeneratorLoops, CanConstructWithMotionAndControllerCallback) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(100));
    EXPECT_CALL(robot, stopMotion(100));
    EXPECT_CALL(robot, stopController());
  }

  EXPECT_NO_THROW(typename TestFixture::Loop(robot,
                                             [](const RobotState&, Duration) {
                                               return Torques({0, 1, 2, 3, 4, 5, 6});
                                             },
                                             std::bind(&TestFixture::createMotion, this)));
}

TYPED_TEST(MotionGeneratorLoops, CanConstructWithMotionCallbackAndControllerMode) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kCartesianImpedance,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, stopMotion(200));
  }

  EXPECT_NO_THROW(typename TestFixture::Loop(robot, ControllerMode::kCartesianImpedance,
                                             std::bind(&TestFixture::createMotion, this)));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  auto motion = this->createMotion();

  RobotState robot_state{};
  Duration duration(1);
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kJointImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  RobotCommand command;
  randomRobotCommand(command);
  EXPECT_TRUE(loop.spinOnce(robot_state, duration, &command.motion));
  EXPECT_THAT(command.motion, this->getField(motion));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithMotionAndControllerCallback) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});
  auto motion = this->createMotion();

  RobotState robot_state{};
  Duration duration(2);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(torques));
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  RobotCommand command;
  randomRobotCommand(command);
  EXPECT_TRUE(loop.spinOnce(robot_state, duration, &command.motion));
  EXPECT_TRUE(loop.spinOnce(robot_state, duration, &command.control));
  EXPECT_THAT(command.motion, this->getField(motion));
  EXPECT_EQ(torques.tau_J, command.control.tau_J_d);
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  RobotState robot_state{};
  Duration duration(3);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(Stop));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  ControllerCommand control_command{};
  EXPECT_TRUE(loop.spinOnce(robot_state, duration, &control_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  MotionGeneratorCommand motion_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, duration, &motion_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;

  RobotState robot_state{};
  Duration duration(4);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(Stop));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kMotorPD,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  MotionGeneratorCommand motion_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, duration, &motion_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_, _)).WillByDefault(Return(this->createMotion()));

  MockControlCallback control_callback;
  RobotState robot_state{};
  Duration duration(5);
  Duration zero_duration(0);
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration)).WillOnce(Return(Stop));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  MotionGeneratorCommand motion_command{};
  EXPECT_TRUE(loop.spinOnce(robot_state, duration, &motion_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  ControllerCommand control_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, duration, &control_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(control_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(Stop)));
  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(MotionGeneratorLoops, GetsCorrectControlTimeStepWithMotionAndControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_, _)).WillByDefault(Return(this->createMotion()));

  MockControlCallback control_callback;
  RobotState robot_state{};
  robot_state.time = Duration(10);
  Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(control_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const RobotState&, Duration duration) -> Torques {
        EXPECT_EQ(ticks.at(control_count), duration.toMSec());

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

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));
  loop();
}

TYPED_TEST(MotionGeneratorLoops, GetsCorrectMotionTimeStepWithMotionAndControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockControlCallback> control_callback;
  Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(zero_torques));

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  RobotState robot_state{};
  robot_state.time = Duration(10);
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(motion_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(
          Invoke([&](const RobotState&, Duration duration) -> typename TestFixture::TMotion {
            EXPECT_EQ(ticks.at(control_count), duration.toMSec());

            if (++control_count == ticks.size()) {
              return Stop;
            }
            return this->createMotion();
          }));
  size_t robot_count = 0;
  EXPECT_CALL(robot, update(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const MotionGeneratorCommand*, const ControllerCommand*) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));
  loop();
}

TYPED_TEST(MotionGeneratorLoops, GetsCorrectTimeStepWithMotionCallback) {
  NiceMock<MockRobotControl> robot;

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  RobotState robot_state{};
  robot_state.time = Duration(10);
  std::array<uint64_t, 5> ticks{{0, 2, 1, 3, 5}};

  size_t control_count = 0;
  EXPECT_CALL(motion_callback, invoke(_, _))
      .Times(ticks.size())
      .WillRepeatedly(
          Invoke([&](const RobotState&, Duration duration) -> typename TestFixture::TMotion {
            EXPECT_EQ(ticks.at(control_count), duration.toMSec());

            if (++control_count == ticks.size()) {
              return Stop;
            }
            return this->createMotion();
          }));
  size_t robot_count = 0;
  EXPECT_CALL(robot, update(_, _))
      .Times(ticks.size())
      .WillRepeatedly(Invoke([&](const MotionGeneratorCommand*, const ControllerCommand*) {
        robot_state.time += Duration(ticks.at(robot_count));
        robot_count++;
        return robot_state;
      }));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kJointImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));
  loop();
}
