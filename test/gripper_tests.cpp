#include <gmock/gmock.h>

#include <atomic>
#include <functional>

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
  GripperMockServer server;

  Gripper gripper("127.0.0.1");
  EXPECT_EQ(1, gripper.serverVersion());
}

TEST(Gripper, ThrowsOnIncompatibleLibraryVersion) {
  GripperMockServer server([](const Connect::Request&) {
    return Connect::Response(0, Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Gripper("127.0.0.1"), IncompatibleVersionException);
}

TEST(Gripper, ThrowsIfConflictingOperationIsRunning) {
  std::atomic_bool run(true);
  std::atomic_bool started_sending(false);

  GripperMockServer server;
  Gripper gripper("127.0.0.1");

  server
      .doForever(
          [&]() {
            bool continue_sending = run.load();
            if (continue_sending) {
              server.sendEmptyState<GripperState>();
            }
            return continue_sending;
          },
          &started_sending)
      .spinOnce();

  while (!started_sending) {
    std::this_thread::yield();
  }

  std::atomic_bool running(false);
  auto thread = std::thread([&]() {
    while (run) {
      gripper.readOnce();
      running = true;
    }
  });

  while (!running) {
    std::this_thread::yield();
  }

  EXPECT_THROW(gripper.readOnce(), franka::InvalidOperationException);

  server.ignoreUdpBuffer();
  run = false;

  if (thread.joinable()) {
    thread.join();
  }
}
