// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <atomic>
#include <functional>

#include <gmock/gmock.h>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <research_interface/gripper/types.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using franka::Gripper;
using franka::IncompatibleVersionException;
using franka::NetworkException;

using research_interface::gripper::Connect;
using research_interface::gripper::GripperState;

TEST(Gripper, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Gripper gripper("127.0.0.1"), NetworkException)
      << "Shut down local gripper service to run tests.";
}

TEST(Gripper, CanPerformHandshake) {
  GripperMockServer server;

  Gripper gripper("127.0.0.1");
  EXPECT_EQ(research_interface::gripper::kVersion, gripper.serverVersion());
}

TEST(Gripper, ThrowsOnIncompatibleLibraryVersion) {
  GripperMockServer server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Gripper("127.0.0.1"), IncompatibleVersionException);
}
