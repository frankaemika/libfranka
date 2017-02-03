#include <gtest/gtest.h>

#include <research_interface/constants.h>

#include <franka/exception.h>
#include <franka/robot_state.h>

#include "robot_impl.h"

#include "mock_server.h"
#include "helpers.h"

using franka::RobotState;
using franka::NetworkException;
using namespace research_interface;

class Robot : public ::franka::Robot {
 public:
   using ::franka::Robot::Impl;
};

TEST(Robot, ThrowsTimeoutIfNoRobotStateArrives) {
  RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.start();

  using namespace std::chrono_literals;
  Robot::Impl robot("127.0.0.1", kCommandPort, 1ms);

  ASSERT_THROW(robot.waitForRobotState(), NetworkException);
}
