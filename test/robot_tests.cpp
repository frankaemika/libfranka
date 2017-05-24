#include <gmock/gmock.h>

#include <functional>

#include <franka/robot.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using franka::Robot;
using franka::RobotState;
using franka::RealtimeConfig;
using franka::Stop;
using franka::Torques;
using franka::NetworkException;
using franka::IncompatibleVersionException;

using research_interface::Connect;

TEST(Robot, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Robot robot("127.0.0.1"), NetworkException)
      << "Shut down local robot service to run tests.";
}

TEST(Robot, CanPerformHandshake) {
  MockServer server;
  server.spinOnce();

  Robot robot("127.0.0.1");
  EXPECT_EQ(1, robot.serverVersion());
}

TEST(Robot, ThrowsOnIncompatibleLibraryVersion) {
  MockServer server;
  server
      .onConnect([](const Connect::Request&) {
        return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
      })
      .spinOnce();

  EXPECT_THROW(Robot robot("127.0.0.1"), IncompatibleVersionException);
}

TEST(Robot, CanReadRobotStateOnce) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.onSendRobotState([&]() { return sent_robot_state; }).spinOnce();

  Robot robot("127.0.0.1");

  const RobotState& received_robot_state = robot.readOnce();
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST(Robot, CanReadRobotState) {
  struct MockCallback {
    MOCK_METHOD1(invoke, bool(const RobotState&));
  };

  MockServer server;
  server.sendEmptyRobotState().spinOnce();

  Robot robot("127.0.0.1");
  MockCallback callback;
  EXPECT_CALL(callback, invoke(_));

  robot.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });
}
