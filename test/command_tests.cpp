#include <franka/robot.h>
#include <gmock/gmock.h>
#include <robot_impl.h>

#include "mock_server.h"

using franka::Robot;
using franka::RobotState;
using franka::RealtimeConfig;
using franka::Torques;
using franka::CommandException;
using franka::IncompatibleVersionException;

using namespace research_interface;

template <typename T>
class Command : public ::testing::Test {
 public:
  using TCommand = T;

  void executeCommand(Robot::Impl& robot);
  typename T::Request getExpected();
  bool compare(const typename T::Request& request_one, const typename T::Request& request_two);
  typename T::Response createResponse(const typename T::Request& request,
                                      const typename T::Status status);
};

template <typename T>
bool Command<T>::compare(const typename T::Request&, const typename T::Request&) {
  return true;
}

template <>
bool Command<GetCartesianLimit>::compare(const GetCartesianLimit::Request& request_one,
                                         const GetCartesianLimit::Request& request_two) {
  return request_one.id == request_two.id;
}

template <>
bool Command<SetControllerMode>::compare(const SetControllerMode::Request& request_one,
                                         const SetControllerMode::Request& request_two) {
  return request_one.mode == request_two.mode;
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
bool Command<SetFToEE>::compare(const SetFToEE::Request& request_one,
                                const SetFToEE::Request& request_two) {
  return request_one.F_T_EE == request_two.F_T_EE;
}

template <>
bool Command<SetLoad>::compare(const SetLoad::Request& request_one,
                               const SetLoad::Request& request_two) {
  return request_one.F_x_Cload == request_two.F_x_Cload &&
         request_one.I_load == request_two.I_load && request_one.m_load == request_two.m_load;
}

template <>
bool Command<SetTimeScalingFactor>::compare(const SetTimeScalingFactor::Request& request_one,
                                            const SetTimeScalingFactor::Request& request_two) {
  return request_one.time_scaling_factor == request_two.time_scaling_factor;
}

template <>
GetCartesianLimit::Request Command<GetCartesianLimit>::getExpected() {
  int32_t limit_id = 3;
  return GetCartesianLimit::Request(limit_id);
}

template <>
SetControllerMode::Request Command<SetControllerMode>::getExpected() {
  return SetControllerMode::Request(SetControllerMode::ControllerMode::kJointPosition);
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
SetFToEE::Request Command<SetFToEE>::getExpected() {
  std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return SetFToEE::Request(pose);
}

template <>
SetLoad::Request Command<SetLoad>::getExpected() {
  double m_load = 1.5;
  std::array<double, 3> F_x_Cload{0.01, 0.01, 0.1};
  std::array<double, 9> I_load{1, 0, 0, 0, 1, 0, 0, 0, 1};
  return SetLoad::Request(m_load, F_x_Cload, I_load);
}

template <>
SetTimeScalingFactor::Request Command<SetTimeScalingFactor>::getExpected() {
  double factor = 0.5;
  return SetTimeScalingFactor::Request(factor);
}

template <>
AutomaticErrorRecovery::Request Command<AutomaticErrorRecovery>::getExpected() {
  return AutomaticErrorRecovery::Request();
}

template <typename T>
void Command<T>::executeCommand(Robot::Impl& robot) {
  robot.executeCommand<T>();
}

template <>
void Command<GetCartesianLimit>::executeCommand(Robot::Impl& robot) {
  int32_t limit_id = 3;
  franka::VirtualWallCuboid cuboid;
  robot.executeCommand<GetCartesianLimit, int32_t, franka::VirtualWallCuboid*>(limit_id, &cuboid);
}

template <>
void Command<SetControllerMode>::executeCommand(Robot::Impl& robot) {
  SetControllerMode::Request request = getExpected();
  robot.executeCommand<SetControllerMode>(request.mode);
}

template <>
void Command<SetCollisionBehavior>::executeCommand(Robot::Impl& robot) {
  SetCollisionBehavior::Request request = getExpected();
  robot.executeCommand<SetCollisionBehavior>(
      request.lower_torque_thresholds_acceleration, request.upper_torque_thresholds_acceleration,
      request.lower_torque_thresholds_nominal, request.upper_torque_thresholds_nominal,
      request.lower_force_thresholds_acceleration, request.upper_force_thresholds_acceleration,
      request.lower_force_thresholds_nominal, request.upper_force_thresholds_nominal);
}

template <>
void Command<SetJointImpedance>::executeCommand(Robot::Impl& robot) {
  SetJointImpedance::Request request = getExpected();
  robot.executeCommand<SetJointImpedance>(request.K_theta);
}

template <>
void Command<SetCartesianImpedance>::executeCommand(Robot::Impl& robot) {
  SetCartesianImpedance::Request request = getExpected();
  robot.executeCommand<SetCartesianImpedance>(request.K_x);
}

template <>
void Command<SetGuidingMode>::executeCommand(Robot::Impl& robot) {
  SetGuidingMode::Request request = getExpected();
  robot.executeCommand<SetGuidingMode>(request.guiding_mode, request.nullspace);
}

template <>
void Command<SetEEToK>::executeCommand(Robot::Impl& robot) {
  SetEEToK::Request request = getExpected();
  robot.executeCommand<SetEEToK>(request.EE_T_K);
}

template <>
void Command<SetFToEE>::executeCommand(Robot::Impl& robot) {
  SetFToEE::Request request = getExpected();
  robot.executeCommand<SetFToEE>(request.F_T_EE);
}

template <>
void Command<SetLoad>::executeCommand(Robot::Impl& robot) {
  SetLoad::Request request = getExpected();
  robot.executeCommand<SetLoad>(request.m_load, request.F_x_Cload, request.I_load);
}

template <>
void Command<SetTimeScalingFactor>::executeCommand(Robot::Impl& robot) {
  SetTimeScalingFactor::Request request = getExpected();
  robot.executeCommand<SetTimeScalingFactor>(request.time_scaling_factor);
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
  std::array<double, 3> object_p_min{-1, 1, 1}, object_p_max{2, 2, 2};
  std::array<double, 16> object_frame{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return GetCartesianLimit::Response(status, object_p_min, object_p_max, object_frame, true);
}

using CommandTypes = ::testing::Types<GetCartesianLimit,
                                      SetControllerMode,
                                      SetCollisionBehavior,
                                      SetJointImpedance,
                                      SetCartesianImpedance,
                                      SetGuidingMode,
                                      SetEEToK,
                                      SetFToEE,
                                      SetLoad,
                                      SetTimeScalingFactor,
                                      AutomaticErrorRecovery>;

TYPED_TEST_CASE(Command, CommandTypes);

TYPED_TEST(Command, CanSendAndReceiveSuccess) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kSuccess);
          })
      .spinOnce();

  EXPECT_NO_THROW(TestFixture::executeCommand(robot));
}

TYPED_TEST(Command, CanSendAndReceiveAbort) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kAborted);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), CommandException);
}

TYPED_TEST(Command, CanSendAndReceiveRejected) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kRejected);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), CommandException);
}

TYPED_TEST(Command, CanSendAndReceivePreempted) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kPreempted);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(robot), CommandException);
}
