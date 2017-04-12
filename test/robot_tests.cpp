#include <gtest/gtest.h>

#include <franka/robot.h>

#include "mock_server.h"
#include "helpers.h"

using namespace franka;
using research_interface::ConnectRequest;
using research_interface::ConnectReply;

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
  server.onConnect([](const ConnectRequest&) {
           return ConnectReply(ConnectReply::Status::kIncompatibleLibraryVersion);
         })
        .spinOnce();

  EXPECT_THROW(Robot robot("127.0.0.1"), IncompatibleVersionException);
}
