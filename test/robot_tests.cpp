#include <gtest/gtest.h>

#include <research_interface/types.h>

#include "franka/robot.h"
#include "robot_impl.h"
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
  server.start();

  Robot robot("127.0.0.1");
  EXPECT_EQ(1, robot.serverVersion());
}

TEST(Robot, ThrowsOnIncompatibleLibraryVersion) {
  MockServer server;
  server.onConnect([](const ConnectRequest&, ConnectReply& reply) {
           reply.status = ConnectReply::Status::kIncompatibleLibraryVersion;
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
