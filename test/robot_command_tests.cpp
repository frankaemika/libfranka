// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/robot.h>
#include <gmock/gmock.h>
#include <robot_impl.h>

#include "helpers.h"
#include "mock_server.h"

using franka::CommandException;
using franka::IncompatibleVersionException;
using franka::Network;
using franka::ProtocolException;
using franka::RealtimeConfig;
using franka::Robot;
using franka::RobotState;
using franka::Torques;

using research_interface::robot::AutomaticErrorRecovery;
using research_interface::robot::Connect;
using research_interface::robot::GetCartesianLimit;
using research_interface::robot::LoadModelLibrary;
using research_interface::robot::Move;
using research_interface::robot::SetCartesianImpedance;
using research_interface::robot::SetCollisionBehavior;
using research_interface::robot::SetEEToK;
using research_interface::robot::SetFilters;
using research_interface::robot::SetGuidingMode;
using research_interface::robot::SetJointImpedance;
using research_interface::robot::SetLoad;
using research_interface::robot::SetNEToEE;
using research_interface::robot::StopMove;

template <typename T>
class Command : public ::testing::Test {
 public:
  using TCommand = T;

  void executeCommand(Robot::Impl& robot);
  typename T::Request getExpected();
  typename T::Status getSuccess();
  bool compare(const typename T::Request& request_one, const typename T::Request& request_two);
  typename T::Response createResponse(const typename T::Request& request,
                                      const typename T::Status status);
};

template <typename T>
class GetterSetterCommand : public Command<T> {};

class MoveCommand : public Command<research_interface::robot::Move>,
                    public ::testing::WithParamInterface<research_interface::robot::Move::Status> {
};

class AutomaticErrorRecoveryCommand
    : public Command<research_interface::robot::AutomaticErrorRecovery>,
      public ::testing::WithParamInterface<
          research_interface::robot::AutomaticErrorRecovery::Status> {};

class StopMoveCommand
    : public Command<research_interface::robot::StopMove>,
      public ::testing::WithParamInterface<research_interface::robot::StopMove::Status> {};

template <typename T>
typename T::Status Command<T>::getSuccess() {
  return T::Status::kSuccess;
}

template <>
typename Move::Status Command<Move>::getSuccess() {
  return Move::Status::kMotionStarted;
}

template <>
bool Command<Move>::compare(const Move::Request& request_one, const Move::Request& request_two) {
  return request_one.controller_mode == request_two.controller_mode &&
         request_one.motion_generator_mode == request_two.motion_generator_mode &&
         request_one.maximum_path_deviation == request_two.maximum_path_deviation &&
         request_one.maximum_goal_pose_deviation == request_two.maximum_goal_pose_deviation;
}

template <>
bool Command<GetCartesianLimit>::compare(const GetCartesianLimit::Request& request_one,
                                         const GetCartesianLimit::Request& request_two) {
  return request_one.id == request_two.id;
}

template <>
bool Command<SetCollisionBehavior>::compare(const SetCollisionBehavior::Request& request_one,
                                            const SetCollisionBehavior::Request& request_two) {
  return request_one.lower_torque_thresholds_acceleration ==
             request_two.lower_torque_thresholds_acceleration &&
         request_one.upper_torque_thresholds_acceleration ==
             request_two.upper_torque_thresholds_acceleration &&
         request_one.upper_torque_thresholds_nominal ==
             request_two.upper_torque_thresholds_nominal &&
         request_one.lower_force_thresholds_acceleration ==
             request_two.lower_force_thresholds_acceleration &&
         request_one.upper_force_thresholds_acceleration ==
             request_two.upper_force_thresholds_acceleration &&
         request_one.lower_force_thresholds_nominal == request_two.lower_force_thresholds_nominal &&
         request_one.upper_force_thresholds_nominal == request_two.upper_force_thresholds_nominal;
}

template <>
bool Command<SetJointImpedance>::compare(const SetJointImpedance::Request& request_one,
                                         const SetJointImpedance::Request& request_two) {
  return request_one.K_theta == request_two.K_theta;
}

template <>
bool Command<SetCartesianImpedance>::compare(const SetCartesianImpedance::Request& request_one,
                                             const SetCartesianImpedance::Request& request_two) {
  return request_one.K_x == request_two.K_x;
}

template <>
bool Command<SetGuidingMode>::compare(const SetGuidingMode::Request& request_one,
                                      const SetGuidingMode::Request& request_two) {
  return request_one.guiding_mode == request_two.guiding_mode &&
         request_one.nullspace == request_two.nullspace;
}

template <>
bool Command<SetEEToK>::compare(const SetEEToK::Request& request_one,
                                const SetEEToK::Request& request_two) {
  return request_one.EE_T_K == request_two.EE_T_K;
}

template <>
bool Command<SetNEToEE>::compare(const SetNEToEE::Request& request_one,
                                 const SetNEToEE::Request& request_two) {
  return request_one.NE_T_EE == request_two.NE_T_EE;
}

template <>
bool Command<SetFilters>::compare(const SetFilters::Request& request_one,
                                  const SetFilters::Request& request_two) {
  return request_one.joint_position_filter_frequency ==
             request_two.joint_position_filter_frequency &&
         request_one.joint_velocity_filter_frequency ==
             request_two.joint_velocity_filter_frequency &&
         request_one.cartesian_position_filter_frequency ==
             request_two.cartesian_position_filter_frequency &&
         request_one.cartesian_velocity_filter_frequency ==
             request_two.cartesian_velocity_filter_frequency &&
         request_one.controller_filter_frequency == request_two.controller_filter_frequency;
}

template <>
bool Command<SetLoad>::compare(const SetLoad::Request& request_one,
                               const SetLoad::Request& request_two) {
  return request_one.F_x_Cload == request_two.F_x_Cload &&
         request_one.I_load == request_two.I_load && request_one.m_load == request_two.m_load;
}

template <>
bool Command<AutomaticErrorRecovery>::compare(const AutomaticErrorRecovery::Request&,
                                              const AutomaticErrorRecovery::Request&) {
  return true;
}

template <>
bool Command<StopMove>::compare(const StopMove::Request&, const StopMove::Request&) {
  return true;
}

template <>
Move::Request Command<Move>::getExpected() {
  return Move::Request(Move::ControllerMode::kJointImpedance,
                       Move::MotionGeneratorMode::kJointVelocity, Move::Deviation(1, 2, 3),
                       Move::Deviation(4, 5, 6));
}

template <>
GetCartesianLimit::Request Command<GetCartesianLimit>::getExpected() {
  int32_t limit_id = 3;
  return GetCartesianLimit::Request(limit_id);
}

template <>
SetCollisionBehavior::Request Command<SetCollisionBehavior>::getExpected() {
  std::array<double, 7> lower_torque_thresholds_acceleration{1, 2, 3, 4, 5, 6, 7};
  std::array<double, 7> lower_torque_thresholds_nominal{7, 6, 5, 4, 3, 2, 1};
  std::array<double, 6> lower_force_thresholds_acceleration{1, 3, 5, 7, 9, 11};
  std::array<double, 6> lower_force_thresholds_nominal{2, 4, 6, 8, 10, 12};
  std::array<double, 7> upper_torque_thresholds_acceleration{2, 2, 4, 4, 6, 6, 7};
  std::array<double, 7> upper_torque_thresholds_nominal{8, 6, 6, 4, 4, 2, 2};
  std::array<double, 6> upper_force_thresholds_acceleration{1, 4, 5, 8, 9, 12};
  std::array<double, 6> upper_force_thresholds_nominal{3, 4, 7, 8, 11, 12};
  return SetCollisionBehavior::Request(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

template <>
SetJointImpedance::Request Command<SetJointImpedance>::getExpected() {
  std::array<double, 7> K_theta{1, 2, 3, 4, 5, 6, 7};
  return SetJointImpedance::Request(K_theta);
}

template <>
SetCartesianImpedance::Request Command<SetCartesianImpedance>::getExpected() {
  std::array<double, 6> K_x{1, 2, 3, 4, 5, 6};
  return SetCartesianImpedance::Request(K_x);
}

template <>
SetGuidingMode::Request Command<SetGuidingMode>::getExpected() {
  std::array<bool, 6> mode{true, true, false, false, true, false};
  bool nullspace = false;
  return SetGuidingMode::Request(mode, nullspace);
}

template <>
SetEEToK::Request Command<SetEEToK>::getExpected() {
  std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return SetEEToK::Request(pose);
}

template <>
SetNEToEE::Request Command<SetNEToEE>::getExpected() {
  std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return SetNEToEE::Request(pose);
}

template <>
SetFilters::Request Command<SetFilters>::getExpected() {
  return SetFilters::Request(1, 10, 100, 100, 1000);
}

template <>
SetLoad::Request Command<SetLoad>::getExpected() {
  double m_load = 1.5;
  std::array<double, 3> F_x_Cload{0.01, 0.01, 0.1};
  std::array<double, 9> I_load{1, 0, 0, 0, 1, 0, 0, 0, 1};
  return SetLoad::Request(m_load, F_x_Cload, I_load);
}

template <>
AutomaticErrorRecovery::Request Command<AutomaticErrorRecovery>::getExpected() {
  return AutomaticErrorRecovery::Request();
}

template <>
StopMove::Request Command<StopMove>::getExpected() {
  return StopMove::Request();
}

template <typename T>
void Command<T>::executeCommand(Robot::Impl& robot) {
  robot.executeCommand<T>(getExpected());
}

template <typename T>
typename T::Response Command<T>::createResponse(const typename T::Request&,
                                                const typename T::Status status) {
  return typename T::Response(status);
}

template <>
GetCartesianLimit::Response Command<GetCartesianLimit>::createResponse(
    const GetCartesianLimit::Request&,
    GetCartesianLimit::Status status) {
  std::array<double, 3> object_world_size{2, 2, 2};
  std::array<double, 16> object_frame{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return GetCartesianLimit::Response(status, object_world_size, object_frame, true);
}

using CommandTypes = ::testing::Types<GetCartesianLimit,
                                      SetCollisionBehavior,
                                      SetJointImpedance,
                                      SetCartesianImpedance,
                                      SetGuidingMode,
                                      SetEEToK,
                                      SetNEToEE,
                                      SetLoad,
                                      SetFilters,
                                      Move,
                                      StopMove,
                                      AutomaticErrorRecovery>;

TYPED_TEST_CASE(Command, CommandTypes);

TYPED_TEST(Command, CanSendAndReceiveSuccess) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, this->getSuccess());
          })
      .spinOnce();

  EXPECT_NO_THROW(TestFixture::executeCommand(robot));
}

TYPED_TEST(Command, CanSendAndReceiveRejected) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request,
                                        TestFixture::TCommand::Status::kCommandNotPossibleRejected);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), CommandException);
}

TYPED_TEST(Command, ThrowsProtocolExceptionIfInvalidResponseReceived) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  typename TestFixture::TCommand::Status invalid_value = static_cast<decltype(invalid_value)>(-1);

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this, invalid_value](const typename TestFixture::TCommand::Request& request) {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, invalid_value);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), ProtocolException);
}

INSTANTIATE_TEST_CASE_P(StopMoveCommandTests,
                        StopMoveCommand,
                        ::testing::Values(StopMove::Status::kCommandNotPossibleRejected,
                                          StopMove::Status::kReflexAborted,
                                          StopMove::Status::kEmergencyAborted,
                                          StopMove::Status::kAborted));

TEST_P(StopMoveCommand, CanReceiveErrorResponses) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  StopMove::Request request;
  server
      .waitForCommand<StopMove>([this](const StopMove::Request& request) -> StopMove::Response {
        EXPECT_TRUE(this->compare(request, this->getExpected()));
        return this->createResponse(request, GetParam());
      })
      .spinOnce();

  EXPECT_THROW(robot.executeCommand<StopMove>(request), CommandException);
}

using GetterSetterCommandTypes = ::testing::Types<SetCollisionBehavior,
                                                  SetJointImpedance,
                                                  SetCartesianImpedance,
                                                  SetEEToK,
                                                  SetNEToEE,
                                                  SetLoad,
                                                  SetFilters,
                                                  GetCartesianLimit>;

TYPED_TEST_CASE(GetterSetterCommand, GetterSetterCommandTypes);

TYPED_TEST(GetterSetterCommand, CanSendAndReceiveInvalidArgument) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request,
                                        TestFixture::TCommand::Status::kInvalidArgumentRejected);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), CommandException);
}

TEST_F(MoveCommand, CanReceiveMotionStarted) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  Move::Request request(Move::ControllerMode::kJointImpedance,
                        Move::MotionGeneratorMode::kJointVelocity, Move::Deviation(1, 2, 3),
                        Move::Deviation(4, 5, 6));

  server
      .waitForCommand<research_interface::robot::Move>(
          [this](const research_interface::robot::Move::Request& request)
              -> research_interface::robot::Move::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, Move::Status::kMotionStarted);
          })
      .spinOnce();

  robot.executeCommand<Move>(request);
}

TEST_P(MoveCommand, CanReceiveErrorResponses) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  Move::Request request(Move::ControllerMode::kJointImpedance,
                        Move::MotionGeneratorMode::kJointVelocity, Move::Deviation(1, 2, 3),
                        Move::Deviation(4, 5, 6));

  server
      .waitForCommand<::Move>([this](const Move::Request& request) -> Move::Response {
        EXPECT_TRUE(this->compare(request, this->getExpected()));
        return this->createResponse(request, GetParam());
      })
      .spinOnce();

  EXPECT_THROW(robot.executeCommand<Move>(request), CommandException);
}

INSTANTIATE_TEST_CASE_P(
    AutomaticErrorRecoveryCommandTests,
    AutomaticErrorRecoveryCommand,
    ::testing::Values(AutomaticErrorRecovery::Status::kCommandNotPossibleRejected,
                      AutomaticErrorRecovery::Status::kReflexAborted,
                      AutomaticErrorRecovery::Status::kEmergencyAborted,
                      AutomaticErrorRecovery::Status::kManualErrorRecoveryRequiredRejected,
                      AutomaticErrorRecovery::Status::kAborted));

TEST_P(AutomaticErrorRecoveryCommand, CanReceiveErrorResponses) {
  RobotMockServer server;
  Robot::Impl robot(
      std::make_unique<franka::Network>("127.0.0.1", research_interface::robot::kCommandPort), 0);

  AutomaticErrorRecovery::Request request;

  server
      .waitForCommand<AutomaticErrorRecovery>([this](const AutomaticErrorRecovery::Request& request)
                                                  -> AutomaticErrorRecovery::Response {
        EXPECT_TRUE(this->compare(request, this->getExpected()));
        return this->createResponse(request, GetParam());
      })
      .spinOnce();

  EXPECT_THROW(robot.executeCommand<AutomaticErrorRecovery>(request), CommandException);
}

INSTANTIATE_TEST_CASE_P(MoveCommandTests,
                        MoveCommand,
                        ::testing::Values(Move::Status::kPreempted,
                                          Move::Status::kStartAtSingularPoseRejected,
                                          Move::Status::kInvalidArgumentRejected,
                                          Move::Status::kReflexAborted,
                                          Move::Status::kEmergencyAborted,
                                          Move::Status::kInputErrorAborted,
                                          Move::Status::kAborted));
