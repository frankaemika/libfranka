// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <franka/exception.h>
#include <franka/vacuum_gripper.h>

#include <chrono>

#include "helpers.h"
#include "mock_server.h"

using franka::CommandException;
using franka::IncompatibleVersionException;
using franka::Network;
using franka::VacuumGripper;
using franka::VacuumGripperState;

using namespace research_interface::vacuum_gripper;

template <typename T>
class VacuumGripperCommand : public ::testing::Test {
 public:
  using TCommand = T;

  bool executeCommand(VacuumGripper& vacuum_gripper);
  typename T::Request getExpected();
  typename T::Status getSuccess();
  bool compare(const typename T::Request& request_one, const typename T::Request& request_two);
  typename T::Response createResponse(const typename T::Request& request,
                                      const typename T::Status status);
};

template <typename T>
typename T::Status VacuumGripperCommand<T>::getSuccess() {
  return T::Status::kSuccess;
}

template <typename T>
bool VacuumGripperCommand<T>::compare(const typename T::Request&, const typename T::Request&) {
  return true;
}

template <>
bool VacuumGripperCommand<Vacuum>::compare(const Vacuum::Request& request_one,
                                           const Vacuum::Request& request_two) {
  return request_one.vacuum == request_two.vacuum && request_one.profile == request_two.profile &&
         request_one.timeout == request_two.timeout;
}

template <>
bool VacuumGripperCommand<DropOff>::compare(const DropOff::Request& request_one,
                                            const DropOff::Request& request_two) {
  return request_one.timeout == request_two.timeout;
}

template <typename T>
typename T::Request VacuumGripperCommand<T>::getExpected() {
  return typename T::Request();
}

template <>
Vacuum::Request VacuumGripperCommand<Vacuum>::getExpected() {
  uint8_t vacuum = 100;
  Profile profile = Profile::kP0;
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
  return Vacuum::Request(vacuum, profile, timeout);
}

template <>
DropOff::Request VacuumGripperCommand<DropOff>::getExpected() {
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
  return DropOff::Request(timeout);
}

template <>
bool VacuumGripperCommand<Vacuum>::executeCommand(VacuumGripper& vacuum_gripper) {
  uint8_t vacuum = 100;
  franka::VacuumGripper::ProductionSetupProfile profile =
      franka::VacuumGripper::ProductionSetupProfile::kP0;
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
  return vacuum_gripper.vacuum(vacuum, timeout, profile);
}

template <>
bool VacuumGripperCommand<DropOff>::executeCommand(VacuumGripper& vacuum_gripper) {
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);
  return vacuum_gripper.dropOff(timeout);
}

template <>
bool VacuumGripperCommand<Stop>::executeCommand(VacuumGripper& vacuum_gripper) {
  return vacuum_gripper.stop();
}

template <typename T>
typename T::Response VacuumGripperCommand<T>::createResponse(const typename T::Request&,
                                                             const typename T::Status status) {
  return typename T::Response(status);
}

using CommandTypes = ::testing::Types<Vacuum, DropOff, Stop>;

TYPED_TEST_CASE(VacuumGripperCommand, CommandTypes);

TYPED_TEST(VacuumGripperCommand, CanSendAndReceiveSuccess) {
  VacuumGripperMockServer server;
  VacuumGripper vacuum_gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, this->getSuccess());
          })
      .spinOnce();

  EXPECT_NO_THROW(TestFixture::executeCommand(vacuum_gripper));
}

TYPED_TEST(VacuumGripperCommand, CanSendAndReceiveFail) {
  VacuumGripperMockServer server;
  VacuumGripper vacuum_gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kFail);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(vacuum_gripper), CommandException);
}

TYPED_TEST(VacuumGripperCommand, CanSendAndReceiveUnsucessful) {
  VacuumGripperMockServer server;
  franka::VacuumGripper vacuum_gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kUnsuccessful);
          })
      .spinOnce();

  EXPECT_FALSE(TestFixture::executeCommand(vacuum_gripper));
}

TYPED_TEST(VacuumGripperCommand, CanSendAndReceiveAborted) {
  VacuumGripperMockServer server;
  VacuumGripper vacuum_gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kAborted);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(vacuum_gripper), CommandException);
}
