#include <gtest/gtest.h>

#include "franka/robot.h"
#include "robot_impl.h"
#include "mock_server.h"
#include "helpers.h"

using namespace franka;
using namespace message_types;

class TestRobot : public franka::Robot {
 public:
   using Robot::Impl;
};

TEST(Robot, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Robot robot("127.0.0.1"), NetworkException);
}

TEST(Robot, CanPerformHandshake) {
  MockServer server;
  server.start();

  Robot robot("127.0.0.1");
  EXPECT_EQ(1, robot.serverVersion());
}

TEST(Robot, ThrowsOnIncompatibleLibraryVersion) {
  MockServer server;
  server.onConnect([](const ConnectRequest&, ConnectReply& reply) {
           reply.status_code = ConnectReply::StatusCode::kIncompatibleLibraryVersion;
         })
        .start();

  EXPECT_THROW(Robot robot("127.0.0.1"), IncompatibleVersionException);
}

TEST(Robot, RobotStateInitializedToZero) {
  MockServer server;
  server.start();

  Robot robot("127.0.0.1");
  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);
}

TEST(Robot, CanReceiveRobotState) {
  RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.onSendRobotState([&]() {
          return sent_robot_state;
        })
        .start();

  Robot robot("127.0.0.1");

  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);

  ASSERT_TRUE(robot.waitForRobotState());
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST(Robot, StopsIfNoRobotStateArrives) {
  RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.start();

  using namespace std::chrono_literals;
  TestRobot::Impl robot("127.0.0.1", TestRobot::Impl::kDefaultPort, 1ms);

  ASSERT_THROW(robot.waitForRobotState(), NetworkException);
}
