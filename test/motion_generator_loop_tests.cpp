#include <functional>

#include <gmock/gmock.h>

#include "motion_generator_loop.h"
#include "motion_traits.h"

#include "mock_robot_control.h"

using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::NiceMock;
using ::testing::_;

using franka::RobotState;
using franka::Stop;
using franka::JointValues;
using franka::JointVelocities;
using franka::CartesianPose;
using franka::CartesianVelocities;
using franka::Torques;

using research_interface::ControllerCommand;
using research_interface::MotionGeneratorCommand;

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

  research_interface::StartMotionGeneratorRequest::Type kMotionGeneratorType = franka::MotionTraits<T>::kMotionGeneratorType;

  T createMotion();
  std::function<T(const RobotState&)> getCallback();
  bool isSameMotion(const T& motion, const MotionGeneratorCommand& motion_command);
};

template <>
JointValues MotionGeneratorLoops<JointValues>::createMotion() {
  return JointValues({0, 1, 2, 3, 4, 5, 6});
}

template <>
bool MotionGeneratorLoops<JointValues>::isSameMotion(const JointValues& joint_values, const MotionGeneratorCommand& motion_command) {
  return joint_values.q == motion_command.q_d;
}

template <>
JointVelocities MotionGeneratorLoops<JointVelocities>::createMotion() {
  return JointVelocities({0, 1, 2, 3, 4, 5, 6});
}

template <>
bool MotionGeneratorLoops<JointVelocities>::isSameMotion(const JointVelocities& joint_velocities, const MotionGeneratorCommand& motion_command) {
  return joint_velocities.dq == motion_command.dq_d;
}

template <>
CartesianPose MotionGeneratorLoops<CartesianPose>::createMotion() {
  return CartesianPose({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
}

template <>
bool MotionGeneratorLoops<CartesianPose>::isSameMotion(const CartesianPose& pose, const MotionGeneratorCommand& motion_command) {
  return pose.O_T_EE == motion_command.O_T_EE_d;
}

template <>
CartesianVelocities MotionGeneratorLoops<CartesianVelocities>::createMotion() {
  return CartesianVelocities({0, 1, 2, 3, 4, 5});
}

template <>
bool MotionGeneratorLoops<CartesianVelocities>::isSameMotion(const CartesianVelocities& cartesian_velocities, const MotionGeneratorCommand& motion_command) {
  return cartesian_velocities.O_dP_EE == motion_command.O_dP_EE_d;
}

using MotionTypes = ::testing::Types<JointValues,
                                     JointVelocities,
                                     CartesianPose,
                                     CartesianVelocities>;
TYPED_TEST_CASE(MotionGeneratorLoops, MotionTypes);

TYPED_TEST(MotionGeneratorLoops, CanConstructWithoutCallbacks) {
  MockRobotControl robot;
  EXPECT_CALL(robot, startController()).Times(0);
  EXPECT_CALL(robot, stopController()).Times(0);

  typename TestFixture::Loop loop(robot,
    typename TestFixture::ControlCallback(),
    typename TestFixture::MotionGeneratorCallback());
}

TYPED_TEST(MotionGeneratorLoops, CanConstructWithMotionCallback) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startMotionGenerator(this->kMotionGeneratorType));
    EXPECT_CALL(robot, stopMotionGenerator());
  }

  typename TestFixture::Loop loop(robot,
    typename TestFixture::ControlCallback(),
    std::bind(&TestFixture::createMotion, this));
}

TYPED_TEST(MotionGeneratorLoops, CanConstructWithBothCallbacks) {
  MockRobotControl robot;
  {
    InSequence _;
    EXPECT_CALL(robot, startController());
    EXPECT_CALL(robot, startMotionGenerator(this->kMotionGeneratorType));
    EXPECT_CALL(robot, stopMotionGenerator());
    EXPECT_CALL(robot, stopController());
  }

  typename TestFixture::Loop loop(robot,
    [](const RobotState&) { return Torques({0,1,2,3,4,5,6}); },
    std::bind(&TestFixture::createMotion, this));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithoutCallbacks) {
  MockRobotControl robot;

  typename TestFixture::Loop loop(robot,
    typename TestFixture::ControlCallback(),
    typename TestFixture::MotionGeneratorCallback());
  EXPECT_TRUE(loop.spinOnce());
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithMotionCallback) {
  NiceMock<MockRobotControl> robot;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  RobotState robot_state;
  EXPECT_CALL(robot, robotStateMock())
    .WillOnce(ReturnRef(robot_state));
  MotionGeneratorCommand motion_command;
  EXPECT_CALL(robot, motionGeneratorCommandMock())
    .WillOnce(ReturnRef(motion_command));

  auto motion = this->createMotion();
  EXPECT_CALL(motion_callback, invoke(_))
    .WillOnce(Return(motion));

  typename TestFixture::Loop loop(robot,
    typename TestFixture::ControlCallback(),
    std::bind(&decltype(motion_callback)::invoke,
              &motion_callback,
              std::placeholders::_1));
  EXPECT_TRUE(loop.spinOnce());

  EXPECT_TRUE(this->isSameMotion(motion, motion_command));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithBothCallbacks) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  RobotState robot_state;
  EXPECT_CALL(robot, robotStateMock())
    .Times(AtLeast(1))
    .WillRepeatedly(ReturnRef(robot_state));
  ControllerCommand controller_command;
  EXPECT_CALL(robot, controllerCommandMock())
    .WillOnce(ReturnRef(controller_command));
  MotionGeneratorCommand motion_command;
  EXPECT_CALL(robot, motionGeneratorCommandMock())
    .WillOnce(ReturnRef(motion_command));

  Torques torques({0, 1, 2, 3, 4, 5, 6});
  EXPECT_CALL(control_callback, invoke(_))
    .WillOnce(Return(torques));
  auto motion = this->createMotion();
  EXPECT_CALL(motion_callback, invoke(_))
    .WillOnce(Return(motion));

  typename TestFixture::Loop loop(robot,
    std::bind(&MockControlCallback::invoke,
              &control_callback,
              std::placeholders::_1),
    std::bind(&decltype(motion_callback)::invoke,
              &motion_callback,
              std::placeholders::_1));
  EXPECT_TRUE(loop.spinOnce());

  EXPECT_EQ(torques.tau_J, controller_command.tau_J_d);
  EXPECT_TRUE(this->isSameMotion(motion, motion_command));
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallback) {
  NiceMock<MockRobotControl> robot;

  ControllerCommand controller_command;
  ON_CALL(robot, controllerCommandMock())
    .WillByDefault(ReturnRef(controller_command));

  RobotState robot_state;
  ON_CALL(robot, robotStateMock())
    .WillByDefault(ReturnRef(robot_state));

  NiceMock<MockControlCallback> control_callback;
  ON_CALL(control_callback, invoke(_))
    .WillByDefault(Return(Torques({0,1,2,3,4,5,6})));

  typename TestFixture::Loop loop(robot,
    std::bind(&MockControlCallback::invoke,
              &control_callback,
              std::placeholders::_1),
    [](const RobotState&) { return Stop; });
  EXPECT_FALSE(loop.spinOnce());
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingControlCallback) {
  NiceMock<MockRobotControl> robot;

  MotionGeneratorCommand motion_command;
  ON_CALL(robot, motionGeneratorCommandMock())
    .WillByDefault(ReturnRef(motion_command));

  RobotState robot_state;
  ON_CALL(robot, robotStateMock())
    .WillByDefault(ReturnRef(robot_state));

  NiceMock<MockMotionCallback<typename TestFixture::TMotion>> motion_callback;
  ON_CALL(motion_callback, invoke(_))
    .WillByDefault(Return(this->createMotion()));

  typename TestFixture::Loop loop(robot,
    [](const RobotState&) { return Stop; },
    std::bind(&decltype(motion_callback)::invoke,
              &motion_callback,
              std::placeholders::_1));
  EXPECT_FALSE(loop.spinOnce());
}
