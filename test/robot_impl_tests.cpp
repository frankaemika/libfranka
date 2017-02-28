#include <gtest/gtest.h>

#include "robot_impl.h"

#include "mock_server.h"
#include "helpers.h"

using franka::RobotState;
using franka::NetworkException;
using franka::MotionGeneratorException;
using MotionGeneratorType = research_interface::StartMotionGeneratorRequest::Type;

class Robot : public ::franka::Robot {
 public:
   using ::franka::Robot::Impl;
};

TEST(Robot, ThrowsTimeoutIfNoRobotStateArrives) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.start();

  using namespace std::chrono_literals;
  Robot::Impl robot("127.0.0.1", research_interface::kCommandPort, 1ms);

  ASSERT_THROW(robot.update(), NetworkException);
}

TEST(Robot, StopsIfControlConnectionClosed) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.start();

  using namespace std::chrono_literals;
  Robot::Impl robot("127.0.0.1", research_interface::kCommandPort, 1ms);

  server.stop();

  ASSERT_FALSE(robot.update());
}

TEST(Robot, CanStartMotionGenerator) {
  MockServer server;
  server.start();

  Robot::Impl robot("127.0.0.1");
  EXPECT_NO_THROW(robot.startMotionGenerator(MotionGeneratorType::kJointVelocity));
}

TEST(Robot, CanNotStartMultipleMotionGenerators) {
  MockServer server;
  server.start();

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
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointVelocity;
      return robot_state;
    })
    .start();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kJointVelocity);

  research_interface::RobotCommand command;
  randomRobotCommand(command);
  robot.motionCommand() = command.motion;
  robot.update();

  testRobotCommandsAreEqual(command.motion, server.lastCommand().motion);
}
