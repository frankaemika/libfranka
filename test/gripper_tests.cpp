#include <gmock/gmock.h>

#include <functional>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <research_interface/gripper/types.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using franka::Gripper;
using franka::GripperState;
using franka::NetworkException;
using franka::IncompatibleVersionException;

using research_interface::gripper::Connect;

TEST(Gripper, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Gripper gripper("127.0.0.1"), NetworkException)
      << "Shut down local gripper service to run tests.";
}

TEST(Gripper, CanPerformHandshake) {
  MockServer<research_interface::gripper::Connect> server;

  Gripper gripper("127.0.0.1");
  EXPECT_EQ(1, gripper.serverVersion());
}

TEST(Gripper, ThrowsOnIncompatibleLibraryVersion) {
  MockServer<research_interface::gripper::Connect> server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Gripper gripper("127.0.0.1"), IncompatibleVersionException);
}

TEST(Gripper, CanReadGripperStateOnce) {
  research_interface::gripper::GripperState sent_gripper_state;
  randomGripperState(sent_gripper_state);

  MockServer<research_interface::gripper::Connect> server;
  Gripper gripper("127.0.0.1");

  server.onSendUDP<research_interface::gripper::GripperState>([&]() { return sent_gripper_state; })
      .spinOnce();

  const GripperState& received_gripper_state = gripper.readOnce();
  testGripperStatesAreEqual(sent_gripper_state, received_gripper_state);
}
