#include <franka/exception.h>
#include <franka/gripper.h>

#include <gmock/gmock.h>

#include "helpers.h"
#include "mock_server.h"

using franka::Network;
using franka::Gripper;
using franka::GripperState;
using franka::CommandException;
using franka::IncompatibleVersionException;

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
  void check(Gripper& gripper);
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
  return request_one.width == request_two.width && request_one.speed == request_two.speed &&
         request_one.force == request_two.force;
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
  double speed = 0.1;
  double force = 400.0;
  return Grasp::Request(width, speed, force);
}

template <>
bool GripperCommand<Move>::executeCommand(Gripper& gripper) {
  double width = 0.05;
  double speed = 0.1;
  gripper.move(width, speed);
  return true;
}

template <>
bool GripperCommand<Grasp>::executeCommand(Gripper& gripper) {
  double width = 0.05;
  double speed = 0.1;
  double force = 400.0;
  return gripper.grasp(width, speed, force);
  ;
}

template <>
bool GripperCommand<Stop>::executeCommand(Gripper& gripper) {
  gripper.stop();
  return true;
}

template <>
bool GripperCommand<Homing>::executeCommand(Gripper& gripper) {
  gripper.homing();
  return true;
}

template <typename T>
typename T::Response GripperCommand<T>::createResponse(const typename T::Request&,
                                                       const typename T::Status status) {
  return typename T::Response(status);
}

template <typename T>
void GripperCommand<T>::check(Gripper& gripper) {
  EXPECT_THROW(executeCommand(gripper), CommandException);
}

template <>
void GripperCommand<Grasp>::check(Gripper& gripper) {
  EXPECT_FALSE(executeCommand(gripper));
}

using CommandTypes = ::testing::Types<Homing, Move, Grasp, Stop>;

TYPED_TEST_CASE(GripperCommand, CommandTypes);

TYPED_TEST(GripperCommand, CanSendAndReceiveSuccess) {
  MockServer<research_interface::gripper::Connect> server;
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
  MockServer<research_interface::gripper::Connect> server;
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
  MockServer<research_interface::gripper::Connect> server;
  Gripper gripper("127.0.0.1");

  server
      .waitForCommand<typename TestFixture::TCommand>(
          [this](const typename TestFixture::TCommand::Request& request) ->
          typename TestFixture::TCommand::Response {
            EXPECT_TRUE(this->compare(request, this->getExpected()));
            return this->createResponse(request, TestFixture::TCommand::Status::kUnsuccessful);
          })
      .spinOnce();

  TestFixture::check(gripper);
}