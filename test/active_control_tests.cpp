// Copyright (c) 2023 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <franka/active_control.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <robot_impl.h>

#include "helpers.h"
#include "mock_robot.h"
#include "mock_robot_impl.h"
#include "mock_server.h"

using ::testing::Matcher;

using namespace research_interface;

using namespace franka;

class ActiveControlTest : public ::testing::Test {
 public:
  ActiveControlTest()
      : robot_impl_mock_(std::make_shared<RobotImplMock>(
            std::move(std::make_unique<Network>("127.0.0.1", robot::kCommandPort)),
            0,
            RealtimeConfig::kIgnore)),
        robot_(RobotMock(robot_impl_mock_)) {
    server_.sendEmptyState<robot::RobotState>().spinOnce();
  };

  std::unique_ptr<ActiveControl<JointVelocities>> startControl() {
    EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce(::testing::Return(100));

    return robot_.startTorqueControl();
  }

  std::unique_ptr<ActiveControl<JointPositions>> startJointPositionsControl() {
    EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_))
        .Times(1)
        .WillOnce(::testing::Return(100));

    return robot_.startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
  }
  //   std::unique_ptr<ActiveControl<JointVelocities>> startControl() {
  //   EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_))
  //       .Times(1)
  //       .WillOnce(::testing::Return(100));

  //   return robot_.startTorqueControl();
  // }
  //   std::unique_ptr<ActiveControl<JointVelocities>> startControl() {
  //   EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_))
  //       .Times(1)
  //       .WillOnce(::testing::Return(100));

  //   return robot_.startTorqueControl();
  // }
  //   std::unique_ptr<ActiveControl<JointVelocities>> startControl() {
  //   EXPECT_CALL(*robot_impl_mock_, startMotion(testing::_, testing::_, testing::_, testing::_))
  //       .Times(1)
  //       .WillOnce(::testing::Return(100));

  //   return robot_.startTorqueControl();
  // }

 protected:
  RobotMockServer server_;
  std::shared_ptr<RobotImplMock> robot_impl_mock_;
  RobotMock robot_;
};

TEST_F(ActiveControlTest, CanWriteOnceIfControlNotFinished) {
  std::unique_ptr<ActiveControl<JointVelocities>> active_control = startControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};

  EXPECT_CALL(*robot_impl_mock_, cancelMotion(100)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, writeOnce(Matcher<const Torques&>(testing::_))).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
}

TEST_F(ActiveControlTest, jointPositionControl) {
  std::unique_ptr<ActiveControl<JointPositions>> active_control = startJointPositionsControl();
  JointPositions default_joint_positions{{1, 1, 1, 1, 1, 1, 1}};

  EXPECT_CALL(*robot_impl_mock_, cancelMotion(100)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, writeOnce(Matcher<const JointPositions&>(testing::_))).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_joint_positions));
}

TEST_F(ActiveControlTest, CanCallFinishMotionWhenFinished) {
  std::unique_ptr<ActiveControl<JointVelocities>> active_control = startControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock_, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
}

TEST_F(ActiveControlTest, CanNotWriteOnceIfControlFinished) {
  std::unique_ptr<ActiveControl<JointVelocities>> active_control = startControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock_, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
  EXPECT_THROW(active_control->writeOnce(default_control_output), ControlException);
}

TEST_F(ActiveControlTest, ControlTokenReleasedAfterFinishingControl) {
  std::unique_ptr<ActiveControl<JointVelocities>> active_control = startControl();
  Torques default_control_output{{0, 0, 0, 0, 0, 0, 0}};
  default_control_output.motion_finished = true;

  EXPECT_CALL(*robot_impl_mock_, finishMotion(100, testing::_, testing::_)).Times(1);
  EXPECT_NO_THROW(active_control->writeOnce(default_control_output));
  EXPECT_CALL(*robot_impl_mock_, cancelMotion(100)).Times(1);
  EXPECT_NO_THROW(startControl());
}

TEST_F(ActiveControlTest, CanReadOnce) {
  std::unique_ptr<ActiveControl<JointVelocities>> active_control = startControl();
  const Duration time_first_read(1);
  const Duration time_second_read(5);
  const RobotState kFirstExpectedRobotState{.time = time_first_read};
  const RobotState kSecondExpectedRobotState{.time = time_second_read};

  EXPECT_CALL(*robot_impl_mock_, readOnce())
      .Times(2)
      .WillOnce(::testing::Return(kFirstExpectedRobotState))
      .WillOnce(::testing::Return(kSecondExpectedRobotState));
  EXPECT_CALL(*robot_impl_mock_, cancelMotion(100)).Times(1);
  auto& first_throw_motion_error_call =
      EXPECT_CALL(*robot_impl_mock_, throwOnMotionError(kFirstExpectedRobotState, 100)).Times(1);
  EXPECT_CALL(*robot_impl_mock_, throwOnMotionError(kSecondExpectedRobotState, 100))
      .Times(1)
      .After(first_throw_motion_error_call);

  auto [received_robot_state, received_period] = active_control->readOnce();
  EXPECT_EQ(received_robot_state, kFirstExpectedRobotState);
  EXPECT_EQ(received_period, Duration(0));

  std::tie(received_robot_state, received_period) = active_control->readOnce();
  EXPECT_EQ(received_robot_state, kSecondExpectedRobotState);
  EXPECT_EQ(received_period, time_second_read - time_first_read);
}
