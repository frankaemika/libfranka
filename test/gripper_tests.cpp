#include <gmock/gmock.h>

#include <functional>
#include <limits>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <research_interface/gripper/types.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using franka::Gripper;
using franka::NetworkException;
using franka::IncompatibleVersionException;

using research_interface::gripper::Connect;
using research_interface::gripper::GripperState;

TEST(Gripper, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Gripper gripper("127.0.0.1"), NetworkException)
      << "Shut down local gripper service to run tests.";
}

TEST(Gripper, CanPerformHandshake) {
  MockServer<Connect> server;

  Gripper gripper("127.0.0.1");
  EXPECT_EQ(1, gripper.serverVersion());
}

TEST(Gripper, ThrowsOnIncompatibleLibraryVersion) {
  MockServer<Connect> server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Gripper gripper("127.0.0.1"), IncompatibleVersionException);
}

TEST(Gripper, CanReadGripperStateOnce) {
  MockServer<Connect> server;
  Gripper gripper("127.0.0.1");

  GripperState sent_gripper_state;
  server.sendEmptyState<GripperState>()
      .sendRandomState<GripperState>([](auto& s) { randomGripperState(s); }, &sent_gripper_state)
      .spinOnce();

  const franka::GripperState& received_gripper_state = gripper.readOnce();
  testGripperStatesAreEqual(sent_gripper_state, received_gripper_state);
}

TEST(Gripper, CanReceiveReorderedGripperStatesCorrectly) {
  MockServer<Connect> server;
  Gripper gripper("127.0.0.1");

  server.onSendUDP<GripperState>([](GripperState& gripper_state) { gripper_state.message_id = 2; })
      .spinOnce();

  auto received_gripper_state = gripper.readOnce();
  EXPECT_EQ(2u, received_gripper_state.sequence_number);

  server.onSendUDP<GripperState>([](GripperState& gripper_state) { gripper_state.message_id = 1; })
      .onSendUDP<GripperState>([](GripperState& gripper_state) { gripper_state.message_id = 4; })
      .onSendUDP<GripperState>([](GripperState& gripper_state) { gripper_state.message_id = 2; })
      .onSendUDP<GripperState>([](GripperState& gripper_state) { gripper_state.message_id = 3; })
      .spinOnce();

  received_gripper_state = gripper.readOnce();
  EXPECT_EQ(4u, received_gripper_state.sequence_number);
}

TEST(Gripper, CanReceiveOverflowingGripperStatesCorrectly) {
  MockServer<Connect> server(MockServer<Connect>::ConnectCallbackT(),
                             std::numeric_limits<uint32_t>::max() - 2);
  Gripper gripper("127.0.0.1");

  server
      .onSendUDP<GripperState>([](GripperState& gripper_state) {
        gripper_state.message_id = std::numeric_limits<uint32_t>::max();
      })
      .spinOnce();
  auto received_gripper_state = gripper.readOnce();
  EXPECT_EQ(std::numeric_limits<uint32_t>::max(), received_gripper_state.sequence_number);

  server
      .onSendUDP<GripperState>([](GripperState& gripper_state) {
        gripper_state.message_id = std::numeric_limits<uint32_t>::max() + 1;
      })
      .spinOnce();
  received_gripper_state = gripper.readOnce();
  EXPECT_EQ(0u, received_gripper_state.sequence_number);

  server
      .onSendUDP<GripperState>([](GripperState& gripper_state) {
        gripper_state.message_id = std::numeric_limits<uint32_t>::max() + 2;
      })
      .spinOnce();
  received_gripper_state = gripper.readOnce();
  EXPECT_EQ(1u, received_gripper_state.sequence_number);
}
