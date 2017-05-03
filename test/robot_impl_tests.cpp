#include <gtest/gtest.h>

#include <cstring>

#include <robot_impl.h>

#include "mock_server.h"
#include "helpers.h"

using franka::RobotState;
using franka::ControlException;
using franka::NetworkException;
using MotionGeneratorType = research_interface::StartMotionGeneratorRequest::Type;
using namespace std::chrono_literals;

class Robot : public ::franka::Robot {
 public:
   using ::franka::Robot::Impl;
};

TEST(Robot, RobotStateInitializedToZero) {
  MockServer server;
  server.spinOnce();

  Robot::Impl robot("127.0.0.1");
  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);
}

TEST(Robot, CanReceiveRobotState) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.onSendRobotState([&]() {
          return sent_robot_state;
        })
        .spinOnce();

  Robot::Impl robot("127.0.0.1");

  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);

  ASSERT_TRUE(robot.update());
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST(Robot, ThrowsTimeoutIfNoRobotStateArrives) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;
  server.spinOnce();

  Robot::Impl robot("127.0.0.1", research_interface::kCommandPort, 1ms);

  ASSERT_THROW(robot.update(), NetworkException);
}

TEST(Robot, StopsIfControlConnectionClosed) {
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
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest& request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kJointVelocity, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointVelocity;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  EXPECT_NO_THROW(robot.startMotionGenerator(MotionGeneratorType::kJointVelocity));
}

TEST(Robot, CanStartController) {
  MockServer server;
  server
    .onStartController([](const research_interface::StartControllerRequest&) {
      return research_interface::StartControllerReply(research_interface::StartControllerReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  EXPECT_NO_THROW(robot.startController());
}

TEST(Robot, CanNotStartMultipleMotionGenerators) {
  MockServer server;
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest& request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kJointPosition, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointPosition;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kJointPosition);
  EXPECT_THROW(robot.startMotionGenerator(MotionGeneratorType::kJointVelocity), ControlException);
}

TEST(Robot, CanNotStartMultipleControllers) {
  MockServer server;
  server
    .onStartController([](const research_interface::StartControllerRequest&) {
      return research_interface::StartControllerReply(research_interface::StartControllerReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startController();
  EXPECT_THROW(robot.startController(), ControlException);
}

TEST(Robot, CanSendMotionGeneratorCommand) {
  research_interface::RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  MockServer server;
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest& request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kCartesianPosition, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianPosition;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kCartesianPosition);

  robot.motionGeneratorCommand(sent_command.motion);

  server
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianPosition;
    })
    .onReceiveRobotCommand([&](const research_interface::RobotCommand& command) {
      testMotionGeneratorCommandsAreEqual(sent_command.motion, command.motion);
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_TRUE(robot.motionGeneratorRunning());
}

TEST(Robot, CanSendControllerCommand) {
  research_interface::RobotCommand sent_command;
  randomRobotCommand(sent_command);

  MockServer server;
  server
    .onStartController([](const research_interface::StartControllerRequest&) {
      return research_interface::StartControllerReply(research_interface::StartControllerReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startController();

  robot.controllerCommand(sent_command.control);

  server
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .onReceiveRobotCommand([&](const research_interface::RobotCommand& command) {
      testControllerCommandsAreEqual(sent_command.control, command.control);
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_TRUE(robot.controllerRunning());
}

TEST(Robot, CanReceiveMotionGenerationError) {
  MockServer server;
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest& request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kCartesianPosition, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianPosition;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kCartesianPosition);

  server
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianPosition;
    })
    .onReceiveRobotCommand([](const research_interface::RobotCommand&) {
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());

  server
    .sendReply<research_interface::StartMotionGeneratorReply>([]() {
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kRejected);
    })
    .spinOnce(/* block until reply has been sent */ true);

  EXPECT_THROW(robot.update(), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(Robot, CanStopMotionGenerator) {
  MockServer server;
  server
    .onStartMotionGenerator([](const research_interface::StartMotionGeneratorRequest& request) {
      EXPECT_EQ(research_interface::StartMotionGeneratorRequest::Type::kCartesianVelocity, request.type);
      return research_interface::StartMotionGeneratorReply(research_interface::StartMotionGeneratorReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianVelocity;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startMotionGenerator(MotionGeneratorType::kCartesianVelocity);

  server
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kCartesianPosition;
    })
    .onReceiveRobotCommand([](const research_interface::RobotCommand&) {
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());

  server
    .onStopMotionGenerator([](const research_interface::StopMotionGeneratorRequest&) {
      return research_interface::StopMotionGeneratorReply(research_interface::StopMotionGeneratorReply::Status::kSuccess);
    })
    .sendEmptyRobotState()
    .spinOnce();

  robot.stopMotionGenerator();

  server
    .sendEmptyRobotState()
    .onReceiveRobotCommand([](const research_interface::RobotCommand&) {
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(Robot, CanStopController) {
  MockServer server;
  server
    .onStartController([](const research_interface::StartControllerRequest&) {
      return research_interface::StartControllerReply(research_interface::StartControllerReply::Status::kSuccess);
    })
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .spinOnce();

  Robot::Impl robot("127.0.0.1");
  robot.startController();

  server
    .onSendRobotState([](research_interface::RobotState& robot_state) {
      robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
    })
    .onReceiveRobotCommand([](const research_interface::RobotCommand&) {
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());

  server
    .onStopController([](const research_interface::StopControllerRequest&) {
      return research_interface::StopControllerReply(research_interface::StopControllerReply::Status::kSuccess);
    })
    .sendEmptyRobotState()
    .spinOnce();

  robot.stopController();

  server
    .sendEmptyRobotState()
    .onReceiveRobotCommand([](const research_interface::RobotCommand&) {
    })
    .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_FALSE(robot.controllerRunning());
}
