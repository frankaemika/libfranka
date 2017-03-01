#include <gtest/gtest.h>

#include <cstring>

#include <robot_impl.h>

#include "mock_server.h"
#include "helpers.h"

using franka::RobotState;
using franka::NetworkException;
using franka::MotionGeneratorException;
using MotionGeneratorType = research_interface::StartMotionGeneratorRequest::Type;
using namespace std::chrono_literals;

class Robot : public ::franka::Robot {
 public:
   using ::franka::Robot::Impl;
};

TEST(Robot, ThrowsTimeoutIfNoRobotStateArrives) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.spinOnce();

  Robot::Impl robot("127.0.0.1", research_interface::kCommandPort, 1ms);

  ASSERT_THROW(robot.update(), NetworkException);
}

TEST(Robot, StopsIfControlConnectionClosed) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  std::unique_ptr<Robot::Impl> robot;
  {
    MockServer server;
    server
      .sendEmptyRobotState()
      .spinOnce();

    robot.reset(new Robot::Impl("127.0.0.1", research_interface::kCommandPort, 1ms));

    EXPECT_TRUE(robot->update());
  }

  EXPECT_FALSE(robot->update());
}

TEST(Robot, CanStartMotionGenerator) {
  MockServer server;
  server.spinOnce();

  Robot::Impl robot("127.0.0.1");
  EXPECT_NO_THROW(robot.startMotionGenerator(MotionGeneratorType::kJointVelocity));
}

TEST(Robot, CanNotStartMultipleMotionGenerators) {
  MockServer server;
  server.spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kJointVelocity);
  EXPECT_THROW(robot.startMotionGenerator(MotionGeneratorType::kJointVelocity), MotionGeneratorException);
}

TEST(Robot, CanSendMotionGeneratorCommand) {
  MockServer server;
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kJointVelocity, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([]() {
      research_interface::RobotState robot_state;
      std::memset(&robot_state, 0, sizeof(robot_state));
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointVelocity;
      return robot_state;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kJointVelocity);

  research_interface::RobotCommand command;
  randomRobotCommand(command);
  robot.motionCommand() = command.motion;
  robot.update();

  testRobotCommandsAreEqual(command.motion, server.lastCommand().motion);
}
