// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <limits>

#include <logger.h>
#include <robot_impl.h>

#include "helpers.h"
#include "mock_server.h"

using namespace std::chrono_literals;
using namespace research_interface::robot;

using franka::ControlException;
using franka::NetworkException;

struct Robot : public ::franka::Robot {
  struct Impl : public ::franka::Robot::Impl {
    using ::franka::Robot::Impl::controllerRunning;
    using ::franka::Robot::Impl::Impl;
    using ::franka::Robot::Impl::motionGeneratorRunning;
  };
};

TEST(RobotImpl, CanReceiveRobotState) {
  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  RobotState sent_robot_state;
  server.sendRandomState<RobotState>([](auto s) { randomRobotState(s); }, &sent_robot_state)
      .spinOnce();

  auto received_robot_state = robot.update(nullptr, nullptr);
  testRobotStatesAreEqual(sent_robot_state, received_robot_state);
}

TEST(RobotImpl, CanReceiveReorderedRobotStatesCorrectly) {
  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server.onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 2; })
      .spinOnce();

  auto received_robot_state = robot.update(nullptr, nullptr);
  EXPECT_EQ(2u, received_robot_state.time.toMSec());

  server.onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 1; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 4; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 2; })
      .onSendUDP<RobotState>([](RobotState& robot_state) { robot_state.message_id = 3; })
      .spinOnce();

  received_robot_state = robot.update(nullptr, nullptr);
  EXPECT_EQ(4u, received_robot_state.time.toMSec());
}

TEST(RobotImpl, ThrowsTimeoutIfNoRobotStateArrives) {
  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort, 200ms), 0);

  EXPECT_THROW(robot.update(nullptr, nullptr), NetworkException);
}

TEST(RobotImpl, StopsIfControlConnectionClosed) {
  std::unique_ptr<Robot::Impl> robot;
  {
    RobotMockServer server;

    robot.reset(
        new Robot::Impl(std::make_unique<franka::Network>("127.0.0.1", kCommandPort, 200ms), 0));

    RobotState robot_state;
    server.sendRandomState<RobotState>([](auto s) { randomRobotState(s); }, &robot_state)
        .spinOnce();

    testRobotStatesAreEqual(franka::convertRobotState(robot_state),
                            robot->update(nullptr, nullptr));
  }

  EXPECT_THROW(robot->update(nullptr, nullptr), NetworkException);
}

TEST(RobotImpl, CanStartMotion) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kJointPosition, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kJointImpedance, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotion(Move::ControllerMode::kJointImpedance,
                                    Move::MotionGeneratorMode::kJointPosition,
                                    maximum_path_deviation, maximum_goal_pose_deviation));
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_FALSE(robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce();
  EXPECT_NO_THROW(robot.update(nullptr, nullptr));

  ControllerCommand control_command{};
  MotionGeneratorCommand motion_command{};
  EXPECT_THROW(robot.update(nullptr, &control_command), ControlException);
  EXPECT_THROW(robot.update(&motion_command, &control_command), ControlException);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();
  EXPECT_NO_THROW(robot.update(&motion_command, nullptr));
}

TEST(RobotImpl, CanStartMotionWithController) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kCartesianPosition, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotion(Move::ControllerMode::kExternalController,
                                    Move::MotionGeneratorMode::kCartesianPosition,
                                    maximum_path_deviation, maximum_goal_pose_deviation));
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce();
  EXPECT_NO_THROW(robot.update(nullptr, nullptr));

  ControllerCommand control_command{};
  MotionGeneratorCommand motion_command{};
  EXPECT_THROW(robot.update(nullptr, &control_command), ControlException);
  EXPECT_THROW(robot.update(&motion_command, nullptr), ControlException);
}

TEST(RobotImpl, CanStartExternalControllerMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  RobotMockServer server;

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kJointVelocity, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotion(Move::ControllerMode::kExternalController,
                                    Move::MotionGeneratorMode::kJointVelocity,
                                    maximum_path_deviation, maximum_goal_pose_deviation));
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  // Test exceptions if wrong update() overload is called
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce();
  EXPECT_NO_THROW(robot.update(nullptr, nullptr));

  ControllerCommand control_command{};
  MotionGeneratorCommand motion_command{};
  EXPECT_THROW(robot.update(&motion_command, nullptr), ControlException);
  // This should also throw, as we're running external joint velocity generator
  EXPECT_THROW(robot.update(nullptr, &control_command), ControlException);
}

TEST(RobotImpl, CanNotStartMultipleMotions) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotion(Move::ControllerMode::kJointImpedance,
                                    Move::MotionGeneratorMode::kJointVelocity,
                                    maximum_path_deviation, maximum_goal_pose_deviation));
  EXPECT_THROW(robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                 Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                                 maximum_goal_pose_deviation),
               ControlException);
  EXPECT_THROW(robot.startMotion(Move::ControllerMode::kExternalController,
                                 Move::MotionGeneratorMode::kJointVelocity, maximum_path_deviation,
                                 maximum_goal_pose_deviation),
               ControlException);
}

TEST(RobotImpl, CanSendMotionGeneratorCommand) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  const uint32_t message_id = 682;

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  robot.startMotion(Move::ControllerMode::kJointImpedance,
                    Move::MotionGeneratorMode::kJointVelocity, maximum_path_deviation,
                    maximum_goal_pose_deviation);

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testMotionGeneratorCommandsAreEqual(sent_command.motion, command.motion);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.update(&sent_command.motion, nullptr));
}

TEST(RobotImpl, CanSendControllerCommand) {
  const uint32_t message_id = 684;

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>([=](const Move::Request& request) {
        EXPECT_EQ(Move::MotionGeneratorMode::kJointVelocity, request.motion_generator_mode);
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
        EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
        EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.startMotion(Move::ControllerMode::kExternalController,
                                    Move::MotionGeneratorMode::kJointVelocity,
                                    maximum_path_deviation, maximum_goal_pose_deviation));
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testControllerCommandsAreEqual(sent_command.control, command.control);
        testMotionGeneratorCommandsAreEqual(sent_command.motion, command.motion);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.update(&sent_command.motion, &sent_command.control));
}

TEST(RobotImpl, CanSendMotionGeneratorAndControlCommand) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  const uint32_t message_id = 687;

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  robot.startMotion(Move::ControllerMode::kExternalController,
                    Move::MotionGeneratorMode::kCartesianPosition, maximum_path_deviation,
                    maximum_goal_pose_deviation);

  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id + 1;
      })
      .spinOnce()
      .onReceiveRobotCommand([=](const RobotCommand& command) {
        EXPECT_EQ(message_id, command.message_id);
        testMotionGeneratorCommandsAreEqual(sent_command.motion, command.motion);
        testControllerCommandsAreEqual(sent_command.control, command.control);
      })
      .spinOnce();

  EXPECT_NO_THROW(robot.update(&sent_command.motion, &sent_command.control));
}

TEST(RobotImpl, CanReceiveMotionRejected) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  {
    RobotMockServer server;
    Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

    server
        .onSendUDP<RobotState>([=](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .waitForCommand<Move>(
            [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
            &move_id)
        .queueResponse<Move>(
            move_id, []() { return Move::Response(Move::Status::kCommandNotPossibleRejected); })
        .doForever([&]() {
          bool continue_sending = send.test_and_set();
          if (continue_sending) {
            server.onSendUDP<RobotState>([](RobotState& robot_state) {
              robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
              robot_state.controller_mode = ControllerMode::kCartesianImpedance;
              robot_state.robot_mode = RobotMode::kIdle;
            });
          }
          return continue_sending;
        })
        .spinOnce();

    EXPECT_THROW(robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                   Move::MotionGeneratorMode::kCartesianVelocity,
                                   maximum_path_deviation, maximum_goal_pose_deviation),
                 ControlException);
    send.clear();
    EXPECT_FALSE(robot.motionGeneratorRunning());
  }
}

TEST(RobotImpl, CanStopMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  robot.update(&sent_command.motion, nullptr);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();

  robot.finishMotion(id, &sent_command.motion, nullptr);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, StopMotionErrorThrowsControlException) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  robot.update(&sent_command.motion, nullptr);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kEmergencyAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();

  EXPECT_THROW(robot.finishMotion(id, &sent_command.motion, nullptr), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CanCancelMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kIdle;
      })
      .spinOnce()
      .waitForCommand<StopMove>([&](const StopMove::Request&) {
        server.sendResponse<Move>(move_id,
                                  []() { return Move::Response(Move::Status::kPreempted); });
        return StopMove::Response(StopMove::Status::kSuccess);
      })
      .spinOnce();

  robot.cancelMotion(id);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CancelMotionErrorThrowsControlException) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kCartesianImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  server
      .waitForCommand<StopMove>([&](const StopMove::Request&) {
        server.sendResponse<Move>(move_id,
                                  []() { return Move::Response(Move::Status::kPreempted); });
        return StopMove::Response(StopMove::Status::kCommandNotPossibleRejected);
      })
      .spinOnce();

  EXPECT_THROW(robot.cancelMotion(id), ControlException);
}

TEST(RobotImpl, CanStopMotionWithController) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  uint32_t move_id;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kExternalController,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  robot.update(&sent_command.motion, &sent_command.control);

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
      .spinOnce();

  robot.finishMotion(id, &sent_command.motion, &sent_command.control);
  EXPECT_FALSE(robot.motionGeneratorRunning());
  EXPECT_FALSE(robot.controllerRunning());

  server
      .onReceiveRobotCommand([](const RobotCommand& command) {
        EXPECT_TRUE(command.motion.motion_generation_finished);
      })
      .spinOnce();
}

TEST(RobotImpl, ThrowsDuringMotionIfErrorReceived) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kJointImpedance,
                              Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  MotionGeneratorCommand motion_command{};
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = robot.update(&motion_command, nullptr);
  EXPECT_THROW(robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, LogMadeIfErrorReceived) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  uint32_t message_id = 682;

  constexpr size_t log_size = 5;
  constexpr size_t commands_sent_in_loop = log_size * 2;
  std::vector<RobotCommand> commands;
  std::vector<RobotState> states;

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), log_size);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([=, &states](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
        states.push_back(robot_state);
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kExternalController,
                              Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  ASSERT_TRUE(robot.motionGeneratorRunning());

  for (size_t i = 0; i < commands_sent_in_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          randomRobotState(robot_state);
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
          states.push_back(robot_state);
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    RobotCommand sent_command;
    randomRobotCommand(sent_command);
    sent_command.motion.motion_generation_finished = false;
    sent_command.message_id = message_id - 1;
    robot.update(&sent_command.motion, &sent_command.control);
    commands.push_back(sent_command);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  RobotCommand last_command;
  randomRobotCommand(last_command);
  last_command.message_id = message_id - 1;
  auto robot_state = robot.update(&last_command.motion, &last_command.control);
  commands.push_back(last_command);

  try {
    robot.throwOnMotionError(robot_state, id);
    FAIL() << "Expected ControlException";
  } catch (const ControlException& exception) {
    EXPECT_EQ(log_size, exception.log.size());
    for (size_t i = 0; i < log_size - 1; i++) {
      // robot.startMotion calls update(), so there's one state more than there are commands
      size_t start_index = commands_sent_in_loop - log_size + 1;
      testRobotStatesAreEqual(franka::convertRobotState(states[start_index + 1 + i]),
                              exception.log[i].state);
      testRobotCommandsAreEqual(commands[start_index + i], exception.log[i].command);
    }
    franka::Record last = exception.log.back();
    testRobotStatesAreEqual(franka::convertRobotState(states.back()), last.state);
    testRobotCommandsAreEqual(last_command, last.command);
  } catch (...) {
    FAIL() << "Expected ControlException";
  }
}

TEST(RobotImpl, LogShowsOnlyTheLastMotion) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};
  uint32_t message_id = 682;

  constexpr size_t log_size = 5;
  constexpr size_t commands_sent_first_loop = log_size * 2;
  constexpr size_t commands_sent_second_loop = log_size - 2;
  std::vector<RobotCommand> commands;
  std::vector<RobotState> states;

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), log_size);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kExternalController,
                              Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  ASSERT_TRUE(robot.motionGeneratorRunning());
  for (size_t i = 0; i < commands_sent_first_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();
    RobotCommand sent_command{};
    sent_command.motion.motion_generation_finished = false;
    sent_command.message_id = message_id - 1;
    robot.update(&sent_command.motion, &sent_command.control);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[8] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  RobotCommand last_command{};
  last_command.message_id = message_id - 1;
  auto robot_state = robot.update(&last_command.motion, &last_command.control);

  EXPECT_THROW(robot.throwOnMotionError(robot_state, id), ControlException);

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  id = robot.startMotion(Move::ControllerMode::kExternalController,
                         Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                         maximum_goal_pose_deviation);
  ASSERT_TRUE(robot.motionGeneratorRunning());

  for (size_t i = 0; i < commands_sent_second_loop; i++) {
    server
        .onSendUDP<RobotState>([&](RobotState& robot_state) {
          randomRobotState(robot_state);
          robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
          robot_state.controller_mode = ControllerMode::kExternalController;
          robot_state.robot_mode = RobotMode::kMove;
          robot_state.message_id = ++message_id;
          states.push_back(robot_state);
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    RobotCommand sent_command;
    randomRobotCommand(sent_command);
    sent_command.motion.motion_generation_finished = false;
    sent_command.message_id = message_id - 1;
    robot.update(&sent_command.motion, &sent_command.control);
    commands.push_back(sent_command);
  }

  server
      .onSendUDP<RobotState>([&](RobotState& robot_state) {
        randomRobotState(robot_state);
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = ++message_id;
        states.push_back(robot_state);
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  randomRobotCommand(last_command);
  last_command.message_id = message_id - 1;
  robot.update(&last_command.motion, &last_command.control);
  commands.push_back(last_command);

  try {
    robot.throwOnMotionError(robot_state, id);
    FAIL() << "Expected ControlException";
  } catch (const ControlException& exception) {
    EXPECT_EQ(commands_sent_second_loop + 1, exception.log.size());
    for (size_t i = 0; i < commands_sent_second_loop; i++) {
      // robot.startMotion calls update(), so there's one state more than there are commands
      testRobotStatesAreEqual(franka::convertRobotState(states[i + 1]), exception.log[i].state);
      testRobotCommandsAreEqual(commands[i], exception.log[i].command);
    }
    franka::Record last = exception.log.back();
    testRobotStatesAreEqual(franka::convertRobotState(states.back()), last.state);
    testRobotCommandsAreEqual(last_command, last.command);
  } catch (...) {
    FAIL() << "Expected ControlException";
  }
}

TEST(RobotImpl, ThrowsDuringControlIfErrorReceived) {
  const uint32_t message_id = 684;

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
        robot_state.message_id = message_id;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [=](const Move::Request& request) {
            EXPECT_EQ(Move::MotionGeneratorMode::kJointVelocity, request.motion_generator_mode);
            EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
            EXPECT_EQ(maximum_path_deviation, request.maximum_path_deviation);
            EXPECT_EQ(maximum_goal_pose_deviation, request.maximum_goal_pose_deviation);
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kExternalController,
                              Move::MotionGeneratorMode::kJointVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  MotionGeneratorCommand motion_command{};
  ControllerCommand control_command{};
  server
      .onSendUDP<RobotState>([=](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
        robot_state.message_id = message_id + 1;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = robot.update(&motion_command, &control_command);
  EXPECT_THROW(robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(robot.controllerRunning());
}

TEST(RobotImpl, CanStartConsecutiveMotion) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  for (int i = 0; i < 3; i++) {
    uint32_t move_id;
    server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .waitForCommand<Move>(
            [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
            &move_id)
        .spinOnce();

    EXPECT_FALSE(robot.motionGeneratorRunning());
    auto id = robot.startMotion(Move::ControllerMode::kCartesianImpedance,
                                Move::MotionGeneratorMode::kCartesianVelocity,
                                maximum_path_deviation, maximum_goal_pose_deviation);
    EXPECT_TRUE(robot.motionGeneratorRunning());

    server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianPosition;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand&) {})
        .spinOnce();

    robot.update(&sent_command.motion, nullptr);
    EXPECT_TRUE(robot.motionGeneratorRunning());

    server
        .onSendUDP<RobotState>([](RobotState& robot_state) {
          robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
          robot_state.controller_mode = ControllerMode::kCartesianImpedance;
          robot_state.robot_mode = RobotMode::kMove;
        })
        .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kSuccess); })
        .spinOnce()
        .onReceiveRobotCommand([](const RobotCommand& command) {
          EXPECT_TRUE(command.motion.motion_generation_finished);
        })
        .spinOnce();

    robot.finishMotion(id, &sent_command.motion, nullptr);
    EXPECT_FALSE(robot.motionGeneratorRunning());
  }
}

TEST(RobotImpl, CanStartConsecutiveMotionAfterError) {
  RobotMockServer server;
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [&](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kJointImpedance,
                              Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());

  MotionGeneratorCommand motion_command{};
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kIdle;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = robot.update(&motion_command, nullptr);
  EXPECT_THROW(robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [=](const Move::Request&) { return Move::Response(Move::Status::kMotionStarted); })
      .spinOnce();

  robot.startMotion(Move::ControllerMode::kJointImpedance,
                    Move::MotionGeneratorMode::kJointPosition, maximum_path_deviation,
                    maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());
}

TEST(RobotImpl, CanStartConsecutiveControlAfterError) {
  Move::Deviation maximum_path_deviation{0, 1, 2};
  Move::Deviation maximum_goal_pose_deviation{3, 4, 5};

  RobotCommand sent_command;
  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  RobotMockServer server;
  Robot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", kCommandPort), 0);

  uint32_t move_id;
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>(
          [](const Move::Request& request) {
            EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);
            return Move::Response(Move::Status::kMotionStarted);
          },
          &move_id)
      .spinOnce();

  auto id = robot.startMotion(Move::ControllerMode::kExternalController,
                              Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                              maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());

  MotionGeneratorCommand motion_command{};
  ControllerCommand control_command{};
  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.controller_mode = ControllerMode::kJointImpedance;
        robot_state.reflex_reason[0] = true;
        robot_state.robot_mode = RobotMode::kReflex;
      })
      .sendResponse<Move>(move_id, []() { return Move::Response(Move::Status::kReflexAborted); })
      .spinOnce()
      .onReceiveRobotCommand([](const RobotCommand&) {})
      .spinOnce();

  auto robot_state = robot.update(&motion_command, &control_command);
  EXPECT_THROW(robot.throwOnMotionError(robot_state, id), ControlException);
  EXPECT_FALSE(robot.motionGeneratorRunning());
  EXPECT_FALSE(robot.controllerRunning());

  randomRobotCommand(sent_command);
  sent_command.motion.motion_generation_finished = false;

  server
      .onSendUDP<RobotState>([](RobotState& robot_state) {
        robot_state.motion_generator_mode = MotionGeneratorMode::kCartesianVelocity;
        robot_state.controller_mode = ControllerMode::kExternalController;
        robot_state.robot_mode = RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([](const Move::Request& request) {
        EXPECT_EQ(Move::ControllerMode::kExternalController, request.controller_mode);

        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  robot.startMotion(Move::ControllerMode::kExternalController,
                    Move::MotionGeneratorMode::kCartesianVelocity, maximum_path_deviation,
                    maximum_goal_pose_deviation);
  EXPECT_TRUE(robot.motionGeneratorRunning());
  EXPECT_TRUE(robot.controllerRunning());
}