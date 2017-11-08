// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "control_loop.h"
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
class ControlLoop : public franka::ControlLoop<T> {
 public:
  using franka::ControlLoop<T>::ControlLoop;
  using franka::ControlLoop<T>::spinMotion;
  using franka::ControlLoop<T>::spinControl;
};

template <typename T>
struct MockMotionCallback {
  MOCK_METHOD2_T(invoke, T(const RobotState&, Duration));
};

struct JointPositionMotion {
  using Motion = JointPositions;
};
struct JointVelocityMotion {
  using Motion = JointVelocities;
};
struct CartesianPoseMotion {
  using Motion = CartesianPose;
};
struct CartesianPoseMotionWithElbow {
  using Motion = CartesianPose;
};
struct CartesianVelocityMotion {
  using Motion = CartesianVelocities;
};
struct CartesianVelocityMotionWithElbow {
  using Motion = CartesianVelocities;
};

template <typename T>
class ControlLoops : public ::testing::Test {
 public:
  using TMotion = typename T::Motion;
  using Loop = ControlLoop<TMotion>;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::robot::Move::MotionGeneratorMode kMotionGeneratorMode =
      franka::MotionGeneratorTraits<TMotion>::kMotionGeneratorMode;

  TMotion createMotion();
  auto getField(const TMotion& values);
};

template <>
JointPositions ControlLoops<JointPositionMotion>::createMotion() {
  return JointPositions({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto ControlLoops<JointPositionMotion>::getField(const JointPositions& values) {
  return Field(&research_interface::robot::MotionGeneratorCommand::q_d, Eq(values.q));
}

template <>
JointVelocities ControlLoops<JointVelocityMotion>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto ControlLoops<JointVelocityMotion>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::robot::MotionGeneratorCommand::dq_d, Eq(velocities.dq));
}

template <>
CartesianPose ControlLoops<CartesianPoseMotion>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
auto ControlLoops<CartesianPoseMotion>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_d, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_d,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
CartesianPose ControlLoops<CartesianPoseMotionWithElbow>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, {0, 1});
}

template <>
auto ControlLoops<CartesianPoseMotionWithElbow>::getField(const CartesianPose& pose) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_T_EE_d, Eq(pose.O_T_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_d, Eq(pose.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotion>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
auto ControlLoops<CartesianVelocityMotion>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_d,
                     Eq(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_d,
                     Eq(std::array<double, 2>({0, 0}))),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(false)));
}

template <>
CartesianVelocities ControlLoops<CartesianVelocityMotionWithElbow>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5}, {0, -1});
}

template <>
auto ControlLoops<CartesianVelocityMotionWithElbow>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return AllOf(Field(&research_interface::robot::MotionGeneratorCommand::O_dP_EE_d,
                     Eq(cartesian_velocities.O_dP_EE)),
               Field(&research_interface::robot::MotionGeneratorCommand::elbow_d,
                     Eq(cartesian_velocities.elbow)),
               Field(&research_interface::robot::MotionGeneratorCommand::valid_elbow, Eq(true)));
}

using MotionTypes = ::testing::Types<JointPositionMotion,
                                     JointVelocityMotion,
                                     CartesianPoseMotion,
                                     CartesianPoseMotionWithElbow,
                                     CartesianVelocityMotion,
                                     CartesianVelocityMotionWithElbow>;
TYPED_TEST_CASE(ControlLoops, MotionTypes);

TYPED_TEST(ControlLoops, CanNotConstructWithoutMotionCallback) {
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

TYPED_TEST(ControlLoops, CanNotConstructWithoutControlCallback) {
  StrictMock<MockRobotControl> robot;

  EXPECT_THROW(typename TestFixture::Loop loop(robot, typename TestFixture::ControlCallback(),
                                               std::bind(&TestFixture::createMotion, this)),
               std::invalid_argument);
}

TYPED_TEST(ControlLoops, CanConstructWithMotionAndControllerCallback) {
  MockRobotControl robot;
  EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                 this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                 TestFixture::Loop::kDefaultDeviation))
      .WillOnce(Return(100));

  EXPECT_NO_THROW(typename TestFixture::Loop(robot,
                                             [](const RobotState&, Duration) {
                                               return Torques({0, 1, 2, 3, 4, 5, 6});
                                             },
                                             std::bind(&TestFixture::createMotion, this)));
}

TYPED_TEST(ControlLoops, CanConstructWithMotionCallbackAndControllerMode) {
  MockRobotControl robot;
  EXPECT_CALL(robot, startMotion(Move::ControllerMode::kCartesianImpedance,
                                 this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                 TestFixture::Loop::kDefaultDeviation))
      .WillOnce(Return(200));

  EXPECT_NO_THROW(typename TestFixture::Loop(robot, ControllerMode::kCartesianImpedance,
                                             std::bind(&TestFixture::createMotion, this)));
}

TYPED_TEST(ControlLoops, SpinOnceWithMotionCallbackAndControllerMode) {
  StrictMock<MockRobotControl> robot;
  EXPECT_CALL(robot, startMotion(Move::ControllerMode::kJointImpedance, this->kMotionGeneratorMode,
                                 TestFixture::Loop::kDefaultDeviation,
                                 TestFixture::Loop::kDefaultDeviation))
      .WillOnce(Return(200));

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
  EXPECT_TRUE(loop.spinMotion(robot_state, duration, &command.motion));
  EXPECT_THAT(command.motion, this->getField(motion));
}

TYPED_TEST(ControlLoops, SpinOnceWithMotionAndControllerCallback) {
  StrictMock<MockRobotControl> robot;
  EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                 this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                 TestFixture::Loop::kDefaultDeviation))
      .WillOnce(Return(200));

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
  EXPECT_TRUE(loop.spinMotion(robot_state, duration, &command.motion));
  EXPECT_TRUE(loop.spinControl(robot_state, duration, &command.control));
  EXPECT_THAT(command.motion, this->getField(motion));
  EXPECT_EQ(torques.tau_J, command.control.tau_J_d);
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingMotionCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, finishMotion(200, _, _));
  }

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  RobotState robot_state{};
  Duration duration(3);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(this->createMotion())));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  ControllerCommand control_command{};
  EXPECT_TRUE(loop.spinControl(robot_state, duration, &control_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  MotionGeneratorCommand motion_command{};

  ASSERT_FALSE(loop.spinMotion(robot_state, duration, &motion_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(this->createMotion()))));

  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(ControlLoops, LoopWithThrowingMotionCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_, _)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  try {
    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

    loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kCartesianImpedance,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, finishMotion(200, _, nullptr));
  }

  RobotState robot_state{};
  Duration duration(4);
  Duration zero_duration(0);
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(this->createMotion())));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kCartesianImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  // Use ASSERT to abort on failure because loop() in next line would block otherwise.
  MotionGeneratorCommand motion_command{};
  ASSERT_FALSE(loop.spinMotion(robot_state, duration, &motion_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(motion_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(this->createMotion()))));
  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(ControlLoops, LoopWithThrowingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kJointImpedance,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  MockMotionCallback<typename TestFixture::TMotion> motion_callback;
  EXPECT_CALL(motion_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  try {
    typename TestFixture::Loop loop(
        robot, ControllerMode::kJointImpedance,
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

    loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, SpinOnceWithFinishingControlCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, finishMotion(200, _, _));
  }

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_, _)).WillByDefault(Return(this->createMotion()));

  MockControlCallback control_callback;
  RobotState robot_state{};
  Duration duration(5);
  Duration zero_duration(0);
  Torques zero_torques{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_CALL(control_callback, invoke(Ref(robot_state), duration))
      .WillOnce(Return(MotionFinished(zero_torques)));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

  MotionGeneratorCommand motion_command{};
  EXPECT_TRUE(loop.spinMotion(robot_state, duration, &motion_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  ControllerCommand control_command{};

  ASSERT_FALSE(loop.spinControl(robot_state, duration, &control_command));

  EXPECT_CALL(robot, update(_, _)).WillOnce(Return(RobotState()));
  EXPECT_CALL(control_callback, invoke(_, zero_duration))
      .WillOnce(DoAll(SaveArg<0>(&robot_state), Return(MotionFinished(zero_torques))));
  loop();

  testRobotStateIsZero(robot_state);
}

TYPED_TEST(ControlLoops, LoopWithThrowingControlCallback) {
  NiceMock<MockRobotControl> robot;
  {
    InSequence s;
    EXPECT_CALL(robot, startMotion(Move::ControllerMode::kExternalController,
                                   this->kMotionGeneratorMode, TestFixture::Loop::kDefaultDeviation,
                                   TestFixture::Loop::kDefaultDeviation))
        .WillOnce(Return(200));
    EXPECT_CALL(robot, cancelMotion(200));
  }

  NiceMock<MockControlCallback> control_callback;
  EXPECT_CALL(control_callback, invoke(_, _)).WillOnce(Throw(std::domain_error("")));

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_, _)).WillByDefault(Return(this->createMotion()));

  try {
    typename TestFixture::Loop loop(
        robot, std::bind(&MockControlCallback::invoke, &control_callback, _1, _2),
        std::bind(&decltype(motion_callback)::invoke, &motion_callback, _1, _2));

    loop();
  } catch (const std::domain_error&) {
  }
}

TYPED_TEST(ControlLoops, GetsCorrectControlTimeStepWithMotionAndControlCallback) {
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
          return MotionFinished(zero_torques);
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

TYPED_TEST(ControlLoops, GetsCorrectMotionTimeStepWithMotionAndControlCallback) {
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
              return MotionFinished(this->createMotion());
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

TYPED_TEST(ControlLoops, GetsCorrectTimeStepWithMotionCallback) {
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
              return MotionFinished(this->createMotion());
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
