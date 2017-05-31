#include <gtest/gtest.h>

#include <cstring>

#include <robot_impl.h>

#include "helpers.h"
#include "mock_server.h"

using franka::RobotState;
using franka::ControlException;
using franka::NetworkException;
using MotionGeneratorMode = research_interface::StartMotionGenerator::MotionGeneratorMode;
using namespace std::chrono_literals;

class Robot : public ::franka::Robot {
 public:
  using ::franka::Robot::Impl;
};

TEST(RobotImpl, RobotStateInitializedToZero) {
  MockServer server;

  Robot::Impl robot("127.0.0.1");
  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);
}

TEST(RobotImpl, CanReceiveRobotState) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;

  Robot::Impl robot("127.0.0.1");

  server.onSendRobotState([&]() { return sent_robot_state; }).spinOnce();

  const RobotState& received_robot_state = robot.robotState();
  testRobotStateIsZero(received_robot_state);

  EXPECT_TRUE(robot.update());
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST(RobotImpl, ThrowsTimeoutIfNoRobotStateArrives) {
  research_interface::RobotState sent_robot_state;
  randomRobotState(sent_robot_state);

  MockServer server;

  Robot::Impl robot("127.0.0.1", research_interface::kCommandPort, 200ms);

  EXPECT_THROW(robot.update(), NetworkException);
}

TEST(RobotImpl, StopsIfControlConnectionClosed) {
  std::unique_ptr<Robot::Impl> robot;
  {
    MockServer server;

    robot.reset(new Robot::Impl("127.0.0.1", research_interface::kCommandPort, 200ms));

    server.sendEmptyRobotState().spinOnce();

    EXPECT_TRUE(robot->update());
  }

  EXPECT_THROW(robot->update(), NetworkException);
}

TEST(RobotImpl, CanStartMotionGenerator) {
  MockServer server;

  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointVelocity;
      })
      .spinOnce()
      .onStartMotionGenerator([](const research_interface::StartMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartMotionGenerator, request.function);
        EXPECT_EQ(research_interface::StartMotionGenerator::MotionGeneratorMode::kJointVelocity,
                  request.mode);
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotionGenerator(MotionGeneratorMode::kJointVelocity));
}

TEST(RobotImpl, CanStartController) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onStartController([](const research_interface::StartController::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartController, request.function);
        return research_interface::StartController::Response(
            research_interface::StartController::Status::kSuccess);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startController());
}

TEST(RobotImpl, CanNotStartMultipleMotionGenerators) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode = research_interface::MotionGeneratorMode::kJointPosition;
      })
      .spinOnce()
      .onStartMotionGenerator([](const research_interface::StartMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartMotionGenerator, request.function);
        EXPECT_EQ(research_interface::StartMotionGenerator::MotionGeneratorMode::kJointPosition,
                  request.mode);
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kMotionStarted);
      })
      .spinOnce();

  robot.startMotionGenerator(MotionGeneratorMode::kJointPosition);
  EXPECT_THROW(robot.startMotionGenerator(MotionGeneratorMode::kJointVelocity), ControlException);
}

TEST(RobotImpl, CanNotStartMultipleControllers) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onStartController([](const research_interface::StartController::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartController, request.function);
        return research_interface::StartController::Response(
            research_interface::StartController::Status::kSuccess);
      })
      .spinOnce();

  robot.startController();
  EXPECT_THROW(robot.startController(), ControlException);
}

TEST(RobotImpl, CanSendMotionGeneratorCommand) {
  research_interface::RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianPosition;
      })
      .spinOnce()
      .onStartMotionGenerator([](const research_interface::StartMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartMotionGenerator, request.function);
        EXPECT_EQ(research_interface::StartMotionGenerator::MotionGeneratorMode::kCartesianPosition,
                  request.mode);
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kMotionStarted);
      })
      .spinOnce();

  robot.startMotionGenerator(MotionGeneratorMode::kCartesianPosition);

  robot.motionGeneratorCommand(sent_command.motion);

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianPosition;
      })
      .spinOnce()
      .onReceiveRobotCommand([&](const research_interface::RobotCommand& command) {
        testMotionGeneratorCommandsAreEqual(sent_command.motion, command.motion);
      })
      .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_TRUE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CanSendControllerCommand) {
  research_interface::RobotCommand sent_command;
  randomRobotCommand(sent_command);

  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onStartController([](const research_interface::StartController::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartController, request.function);
        return research_interface::StartController::Response(
            research_interface::StartController::Status::kSuccess);
      })
      .spinOnce();

  robot.startController();

  robot.controllerCommand(sent_command.control);

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onReceiveRobotCommand([&](const research_interface::RobotCommand& command) {
        testControllerCommandsAreEqual(sent_command.control, command.control);
      })
      .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_TRUE(robot.controllerRunning());
}

TEST(RobotImpl, CanReceiveMotionGenerationError) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianPosition;
      })
      .spinOnce()
      .onStartMotionGenerator([](const research_interface::StartMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartMotionGenerator, request.function);
        EXPECT_EQ(research_interface::StartMotionGenerator::MotionGeneratorMode::kCartesianPosition,
                  request.mode);
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kMotionStarted);
      })
      .spinOnce();

  robot.startMotionGenerator(MotionGeneratorMode::kCartesianPosition);

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianPosition;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const research_interface::RobotCommand&) {})
      .spinOnce();

  EXPECT_TRUE(robot.update());

  server
      .sendResponse<research_interface::StartMotionGenerator::Response>([]() {
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kRejected);
      })
      .spinOnce();

  EXPECT_THROW(robot.update(), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CanStopMotionGenerator) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianVelocity;
      })
      .spinOnce()
      .onStartMotionGenerator([](const research_interface::StartMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartMotionGenerator, request.function);
        EXPECT_EQ(research_interface::StartMotionGenerator::MotionGeneratorMode::kCartesianVelocity,
                  request.mode);
        return research_interface::StartMotionGenerator::Response(
            research_interface::StartMotionGenerator::Status::kMotionStarted);
      })
      .spinOnce();

  robot.startMotionGenerator(MotionGeneratorMode::kCartesianVelocity);

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.motion_generator_mode =
            research_interface::MotionGeneratorMode::kCartesianPosition;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const research_interface::RobotCommand&) {})
      .spinOnce();

  EXPECT_TRUE(robot.update());

  server.sendEmptyRobotState()
      .spinOnce()
      .onStopMotionGenerator([](const research_interface::StopMotionGenerator::Request& request) {
        EXPECT_EQ(research_interface::Function::kStopMotionGenerator, request.function);
        return research_interface::StopMotionGenerator::Response(
            research_interface::StopMotionGenerator::Status::kSuccess);
      })
      .spinOnce();

  robot.stopMotionGenerator();

  server.sendEmptyRobotState()
      .onReceiveRobotCommand([](const research_interface::RobotCommand&) {})
      .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CanStopController) {
  MockServer server;
  Robot::Impl robot("127.0.0.1");

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onStartController([](const research_interface::StartController::Request& request) {
        EXPECT_EQ(research_interface::Function::kStartController, request.function);
        return research_interface::StartController::Response(
            research_interface::StartController::Status::kSuccess);
      })
      .spinOnce();

  robot.startController();

  server
      .onSendRobotState([](research_interface::RobotState& robot_state) {
        robot_state.controller_mode = research_interface::ControllerMode::kExternalController;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const research_interface::RobotCommand&) {})
      .spinOnce();

  EXPECT_TRUE(robot.update());

  server.sendEmptyRobotState()
      .spinOnce()
      .onStopController([](const research_interface::StopController::Request& request) {
        EXPECT_EQ(research_interface::Function::kStopController, request.function);
        return research_interface::StopController::Response(
            research_interface::StopController::Status::kSuccess);
      })
      .spinOnce();

  robot.stopController();

  server.sendEmptyRobotState()
      .spinOnce()
      .onReceiveRobotCommand([](const research_interface::RobotCommand&) {})
      .spinOnce();

  EXPECT_TRUE(robot.update());
  EXPECT_FALSE(robot.controllerRunning());
}
