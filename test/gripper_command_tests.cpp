// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gmock/gmock.h>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "helpers.h"
#include "mock_server.h"

using franka::CommandException;
using franka::Gripper;
using franka::GripperState;
using franka::IncompatibleVersionException;
using franka::Network;

using namespace research_interface::gripper;

template <typename T>
class GripperCommand : public ::testing::Test {
 public:
  using TCommand = T;

  bool executeCommand(Gripper& gripper);
  typename T::Request getExpected();
  typename T::Status getSuccess();
  bool compare(const typename T::Request& request_one, const typename T::Request& request_two);
  typename T::Response createResponse(const typename T::Request& request,
                                      const typename T::Status status);
};

template <typename T>
typename T::Status GripperCommand<T>::getSuccess() {
  return T::Status::kSuccess;
}

template <typename T>
bool GripperCommand<T>::compare(const typename T::Request&, const typename T::Request&) {
  return true;
}

template <>
bool GripperCommand<Grasp>::compare(const Grasp::Request& request_one,
                                    const Grasp::Request& request_two) {
  return request_one.width == request_two.width &&
         request_one.epsilon.inner == request_two.epsilon.inner &&
         request_one.epsilon.outer == request_two.epsilon.outer &&
         request_one.speed == request_two.speed && request_one.force == request_two.force;
}

template <>
bool GripperCommand<Move>::compare(const Move::Request& request_one,
                                   const Move::Request& request_two) {
  return request_one.width == request_two.width && request_one.speed == request_two.speed;
}

template <typename T>
typename T::Request GripperCommand<T>::getExpected() {
  return typename T::Request();
}

template <>
Move::Request GripperCommand<Move>::getExpected() {
  double width = 0.05;
  double speed = 0.1;
  return Move::Request(width, speed);
}

template <>
Grasp::Request GripperCommand<Grasp>::getExpected() {
  double width = 0.05;
  double epsilon_inner = 0.004;
  double epsilon_outer = 0.005;
  double speed = 0.1;
  double force = 400.0;
  return Grasp::Request(width, {epsilon_inner, epsilon_outer}, speed, force);
}

template <>
bool GripperCommand<Move>::executeCommand(Gripper& gripper) {
  double width = 0.05;
  double speed = 0.1;
  return gripper.move(width, speed);
}

template <>
bool GripperCommand<Grasp>::executeCommand(Gripper& gripper) {
  double width = 0.05;
  double epsilon_inner = 0.004;
  double epsilon_outer = 0.005;
  double speed = 0.1;
  double force = 400.0;
  return gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer);
}

template <>
bool GripperCommand<Stop>::executeCommand(Gripper& gripper) {
  return gripper.stop();
}

template <>
bool GripperCommand<Homing>::executeCommand(Gripper& gripper) {
  return gripper.homing();
}

template <typename T>
typename T::Response GripperCommand<T>::createResponse(const typename T::Request&,
                                                       const typename T::Status status) {
  return typename T::Response(status);
}

using CommandTypes = ::testing::Types<Homing, Move, Grasp, Stop>;

TYPED_TEST_CASE(GripperCommand, CommandTypes);

TYPED_TEST(GripperCommand, CanSendAndReceiveSuccess) {
  GripperMockServer server;
  Gripper gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, this->getSuccess());
          })
      .spinOnce();

  EXPECT_NO_THROW(TestFixture::executeCommand(gripper));
}

TYPED_TEST(GripperCommand, CanSendAndReceiveFail) {
  GripperMockServer server;
  Gripper gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kFail);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(gripper), CommandException);
}

TYPED_TEST(GripperCommand, CanSendAndReceiveUnsucessful) {
  GripperMockServer server;
  Gripper gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kUnsuccessful);
          })
      .spinOnce();

  EXPECT_FALSE(TestFixture::executeCommand(gripper));
}

TYPED_TEST(GripperCommand, CanSendAndReceiveAborted) {
  GripperMockServer server;
  Gripper gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kAborted);
          })
      .spinOnce();

  EXPECT_THROW(TestFixture::executeCommand(gripper), CommandException);
}
