#include <exception>
#include <functional>

#include <gmock/gmock.h>

#include "motion_generator_loop.h"
#include "motion_generator_traits.h"

#include "mock_robot_control.h"

using namespace ::testing;

using franka::CartesianPose;
using franka::CartesianVelocities;
using franka::ControllerMode;
using franka::JointPositions;
using franka::JointVelocities;
using franka::RobotState;
using franka::Stop;
using franka::Torques;

using research_interface::ControllerCommand;
using research_interface::MotionGeneratorCommand;
using research_interface::Move;
using research_interface::RobotCommand;

class MockControlCallback {
 public:
  MOCK_METHOD1(invoke, Torques(const RobotState&));
};

template <typename T>
class MotionGeneratorLoop : public franka::MotionGeneratorLoop<T> {
 public:
  using franka::MotionGeneratorLoop<T>::MotionGeneratorLoop;
  using franka::MotionGeneratorLoop<T>::spinOnce;
};

template <typename T>
struct MockMotionCallback {
  MOCK_METHOD1_T(invoke, T(const RobotState&));
};

template <typename T>
class MotionGeneratorLoops : public ::testing::Test {
 public:
  using Loop = MotionGeneratorLoop<T>;
  using TMotion = T;
  using MotionGeneratorCallback = typename Loop::MotionGeneratorCallback;
  using ControlCallback = typename Loop::ControlCallback;

  const research_interface::Move::MotionGeneratorMode kMotionGeneratorMode =
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
  return Field(&research_interface::MotionGeneratorCommand::q_d, Eq(values.q));
}

template <>
JointVelocities MotionGeneratorLoops<JointVelocities>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto MotionGeneratorLoops<JointVelocities>::getField(const JointVelocities& velocities) {
  return Field(&research_interface::MotionGeneratorCommand::dq_d, Eq(velocities.dq));
}

template <>
CartesianPose MotionGeneratorLoops<CartesianPose>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
auto MotionGeneratorLoops<CartesianPose>::getField(const CartesianPose& pose) {
  return Field(&research_interface::MotionGeneratorCommand::O_T_EE_d, Eq(pose.O_T_EE));
}

template <>
CartesianVelocities MotionGeneratorLoops<CartesianVelocities>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
auto MotionGeneratorLoops<CartesianVelocities>::getField(
    const CartesianVelocities& cartesian_velocities) {
  return Field(&research_interface::MotionGeneratorCommand::O_dP_EE_d,
               Eq(cartesian_velocities.O_dP_EE));
}

namespace research_interface {

bool operator==(const Move::Deviation& left, const Move::Deviation& right) {
  return left.translation == right.translation && left.rotation == right.rotation &&
         left.elbow == right.elbow;
}

}  // namespace research_interface

using MotionTypes =
    ::testing::Types<JointPositions, JointVelocities, CartesianPose, CartesianVelocities>;
TYPED_TEST_CASE(MotionGeneratorLoops, MotionTypes);

TYPED_TEST(MotionGeneratorLoops, CanNotConstructWithoutMotionCallback) {
  StrictMock<MockRobotControl> robot;

  EXPECT_THROW(typename TestFixture::Loop loop(robot,
                                               [](const RobotState&) {
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
                                   TestFixture::Loop::kDefaultDeviation));
    EXPECT_CALL(robot, stopMotion());
  }

  EXPECT_NO_THROW(typename TestFixture::Loop(robot,
                                             [](const RobotState&) {
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
                                   TestFixture::Loop::kDefaultDeviation));
    EXPECT_CALL(robot, stopMotion());
  }

  EXPECT_NO_THROW(typename TestFixture::Loop(robot, ControllerMode::kCartesianImpedance,
                                             std::bind(&TestFixture::createMotion, this)));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  auto motion = this->createMotion();

  RobotState robot_state{};
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state))).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, ControllerMode::kJointImpedance,
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, std::placeholders::_1));

  RobotCommand command;
  randomRobotCommand(command);
  EXPECT_TRUE(loop.spinOnce(robot_state, &command.motion));
  EXPECT_TRUE(loop.spinOnce(robot_state, &command.control));
  EXPECT_THAT(command.motion, this->getField(motion));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithMotionAndControllerCallback) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});
  auto motion = this->createMotion();

  RobotState robot_state{};
  EXPECT_CALL(control_callback, invoke(Ref(robot_state))).WillOnce(Return(torques));
  EXPECT_CALL(motion_callback, invoke(Ref(robot_state))).WillOnce(Return(motion));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1),
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, std::placeholders::_1));

  RobotCommand command;
  randomRobotCommand(command);
  EXPECT_TRUE(loop.spinOnce(robot_state, &command.motion));
  EXPECT_TRUE(loop.spinOnce(robot_state, &command.control));
  EXPECT_THAT(command.motion, this->getField(motion));
  EXPECT_EQ(torques.tau_J, command.control.tau_J_d);
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  typename TestFixture::Loop loop(
      robot, std::bind(&MockControlCallback::invoke, &control_callback, std::placeholders::_1),
      [](const RobotState&) { return Stop; });

  RobotState robot_state{};
  ControllerCommand control_command{};
  EXPECT_TRUE(loop.spinOnce(robot_state, &control_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  MotionGeneratorCommand motion_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, &motion_command));
  loop();
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallbackAndControllerMode) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_)).WillByDefault(Return(Torques({0, 1, 2, 3, 4, 5, 6})));

  typename TestFixture::Loop loop(robot, ControllerMode::kMotorPD,
                                  [](const RobotState&) { return Stop; });

  RobotState robot_state{};
  ControllerCommand control_command{};
  EXPECT_TRUE(loop.spinOnce(robot_state, &control_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  MotionGeneratorCommand motion_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, &motion_command));
  loop();
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingControlCallback) {
  NiceMock<MockRobotControl> robot;

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_)).WillByDefault(Return(this->createMotion()));

  typename TestFixture::Loop loop(
      robot, [](const RobotState&) { return Stop; },
      std::bind(&decltype(motion_callback)::invoke, &motion_callback, std::placeholders::_1));

  RobotState robot_state{};
  MotionGeneratorCommand motion_command{};
  EXPECT_TRUE(loop.spinOnce(robot_state, &motion_command));

  // Use ASSERT to abort on failure because loop() in next line
  // would block otherwise
  ControllerCommand control_command{};
  ASSERT_FALSE(loop.spinOnce(robot_state, &control_command));
  loop();
}
