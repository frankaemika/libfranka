// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <atomic>
#include <functional>

#include <gmock/gmock.h>

#include <franka/exception.h>
#include <franka/vacuum_gripper.h>
#include <research_interface/vacuum_gripper/types.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using franka::IncompatibleVersionException;
using franka::NetworkException;
using franka::VacuumGripper;

using research_interface::vacuum_gripper::Connect;
using research_interface::vacuum_gripper::VacuumGripperState;

TEST(VacuumGripper, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(VacuumGripper vacuum_gripper("127.0.0.1"), NetworkException)
      << "Shut down local vacuum gripper service to run tests.";
}

TEST(VacuumGripper, CanPerformHandshake) {
  VacuumGripperMockServer server;

  VacuumGripper vacuum_gripper("127.0.0.1");
  EXPECT_EQ(research_interface::vacuum_gripper::kVersion, vacuum_gripper.serverVersion());
}

TEST(VacuumGripper, ThrowsOnIncompatibleLibraryVersion) {
  VacuumGripperMockServer server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(VacuumGripper("127.0.0.1"), IncompatibleVersionException);
}
