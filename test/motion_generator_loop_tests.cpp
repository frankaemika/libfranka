#include <functional>

#include <gmock/gmock.h>

#include "motion_generator_loop.h"
#include "motion_traits.h"

#include "mock_robot_control.h"

using ::testing::AtLeast;
using ::testing::Eq;
using ::testing::Field;
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
  auto getField(const T& values);
};

template <>
JointValues MotionGeneratorLoops<JointValues>::createMotion() {
  return JointValues({0, 1, 2, 3, 4, 5, 6});
}

template <>
auto MotionGeneratorLoops<JointValues>::getField(const JointValues& values) {
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
auto MotionGeneratorLoops<CartesianVelocities>::getField(const CartesianVelocities& cartesian_velocities) {
  return Field(&research_interface::MotionGeneratorCommand::O_dP_EE_d, Eq(cartesian_velocities.O_dP_EE));
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

  auto motion = this->createMotion();

  RobotState robot_state;
  EXPECT_CALL(robot, robotStateMock())
    .WillOnce(ReturnRef(robot_state));
  EXPECT_CALL(robot, motionGeneratorCommandMock(this->getField(motion)));

  EXPECT_CALL(motion_callback, invoke(_))
    .WillOnce(Return(motion));

  typename TestFixture::Loop loop(robot,
    typename TestFixture::ControlCallback(),
    std::bind(&decltype(motion_callback)::invoke,
              &motion_callback,
              std::placeholders::_1));
  EXPECT_TRUE(loop.spinOnce());
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithBothCallbacks) {
  NiceMock<MockRobotControl> robot;
  MockControlCallback control_callback;
  MockMotionCallback<typename TestFixture::TMotion> motion_callback;

  Torques torques({0, 1, 2, 3, 4, 5, 6});
  auto motion = this->createMotion();

  RobotState robot_state;
  EXPECT_CALL(robot, robotStateMock())
    .Times(AtLeast(1))
    .WillRepeatedly(ReturnRef(robot_state));
  EXPECT_CALL(robot, motionGeneratorCommandMock(this->getField(motion)));
  EXPECT_CALL(robot, controllerCommandMock(Field(&research_interface::ControllerCommand::tau_J_d, Eq(torques.tau_J))));

  EXPECT_CALL(control_callback, invoke(_))
    .WillOnce(Return(torques));
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
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingMotionCallback) {
  NiceMock<MockRobotControl> robot;

  ON_CALL(robot, controllerCommandMock(_))
    .WillByDefault(Return());

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
  ASSERT_FALSE(loop.spinOnce());
  loop();
}

TYPED_TEST(MotionGeneratorLoops, SpinOnceWithStoppingControlCallback) {
  NiceMock<MockRobotControl> robot;

  ON_CALL(robot, motionGeneratorCommandMock(_))
    .WillByDefault(Return());
  ON_CALL(robot, update())
    .WillByDefault(Return(true));

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
  ASSERT_FALSE(loop.spinOnce());
  loop();
}
