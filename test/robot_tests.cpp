#include <gtest/gtest.h>

#include <franka/robot.h>

#include "mock_server.h"
#include "helpers.h"

using namespace franka;
using research_interface::ConnectRequest;
using research_interface::ConnectReply;

TEST(Robot, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Robot robot("127.0.0.1"), NetworkException);
}

TEST(Robot, CanPerformHandshake) {
  MockServer server;
  server.spinOnce();

  Robot robot("127.0.0.1");
  EXPECT_EQ(1, robot.serverVersion());
}

TEST(Robot, ThrowsOnIncompatibleLibraryVersion) {
  MockServer server;
  server.onConnect([](const ConnectRequest&) {
           return ConnectReply(ConnectReply::Status::kIncompatibleLibraryVersion);
         })
        .spinOnce();

  EXPECT_THROW(Robot robot("127.0.0.1"), IncompatibleVersionException);
}

TEST(Robot, RobotStateInitializedToZero) {
  MockServer server;
  server.spinOnce();

  Robot robot("127.0.0.1");
  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);
}

TEST(Robot, CanReceiveRobotState) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.onSendRobotState([&]() {
          return sent_robot_state;
        })
        .spinOnce();

  Robot robot("127.0.0.1");

  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);

  ASSERT_TRUE(robot.update());
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}