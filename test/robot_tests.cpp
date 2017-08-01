#include <gmock/gmock.h>

#include <atomic>
#include <functional>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using research_interface::robot::Connect;
using research_interface::robot::Move;
using namespace research_interface;

using namespace franka;

TEST(Robot, CannotConnectIfNoServerRunning) {
  EXPECT_THROW(Robot robot("127.0.0.1"), NetworkException)
      << "Shut down local robot service to run tests.";
}

TEST(Robot, CanPerformHandshake) {
  RobotMockServer server;

  Robot robot("127.0.0.1");
  EXPECT_EQ(1, robot.serverVersion());
}

TEST(Robot, ThrowsOnIncompatibleLibraryVersion) {
  RobotMockServer server([](const Connect::Request& request) {
    return Connect::Response(request.header.command_id,
                             Connect::Status::kIncompatibleLibraryVersion);
  });

  EXPECT_THROW(Robot robot("127.0.0.1"), IncompatibleVersionException);
}

TEST(Robot, CanReadRobotState) {
  struct MockCallback {
    MOCK_METHOD1(invoke, bool(const RobotState&));
  };

  RobotMockServer server;
  Robot robot("127.0.0.1");

  server.sendEmptyState<research_interface::robot::RobotState>().spinOnce();

  MockCallback callback;
  EXPECT_CALL(callback, invoke(_));

  robot.read([&](const RobotState& robot_state) { return callback.invoke(robot_state); });
}

TEST(Robot, CanControlRobot) {
  RobotMockServer server;
  Robot robot("127.0.0.1", RealtimeConfig::kIgnore);

  uint32_t move_command_id;

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  uint32_t stopped_message_id = 0;
  server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
        robot_state.robot_mode = robot::RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([&](const Move::Request& request) {
        move_command_id = request.header.command_id;
        server
            .doForever([&]() {
              bool continue_sending = send.test_and_set();
              if (continue_sending) {
                server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                  robot_state.motion_generator_mode = robot::MotionGeneratorMode::kJointPosition;
                  robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
                  robot_state.robot_mode = robot::RobotMode::kMove;
                });
              }
              return continue_sending;
            })
            .onSendUDP<robot::RobotState>([&](robot::RobotState& robot_state) {
              robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
              robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
              robot_state.robot_mode = robot::RobotMode::kIdle;
              stopped_message_id = robot_state.message_id;
            })
            .sendResponse<Move::Response>(
                [&]() { return Move::Response(move_command_id, Move::Status::kSuccess); });
        return Move::Response(move_command_id, Move::Status::kMotionStarted);
      })
      .spinOnce();

  JointPositions joint_positions{{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  int count = 0;
  robot.control(
      [&](const RobotState&, Duration time_step) -> JointPositions {
        if (count == 0) {
          EXPECT_EQ(0u, time_step.ms());
        } else {
          EXPECT_GE(time_step.ms(), 1u);
        }
        if (++count < 5) {
          return joint_positions;
        }
        send.clear();
        return Stop;
      },
      ControllerMode::kJointImpedance);

  ASSERT_NE(0u, stopped_message_id);
  ASSERT_EQ(5, count);

  // Receive the robot commands sent in the motion loop.
  for (int i = 0; i < count - 1; i++) {
    server
        .onReceiveRobotCommand([&](const robot::RobotCommand& robot_command) {
          EXPECT_EQ(joint_positions.q, robot_command.motion.q_d);
          EXPECT_FALSE(robot_command.motion.motion_generation_finished);
          EXPECT_LT(robot_command.message_id, stopped_message_id);
        })
        .spinOnce();
  }

  // Receive the robot commands sent after Stop has been returned from the motion loop.
  // These will be sent at least once and until Robot received the robot state showing the
  // stopped motion.
  server
      .onReceiveRobotCommand([&](const robot::RobotCommand& robot_command) {
        EXPECT_TRUE(robot_command.motion.motion_generation_finished);
        EXPECT_LT(robot_command.message_id, stopped_message_id);
      })
      .spinOnce();

  // Ignore remaining RobotCommands that might have been sent to the server.
  server.ignoreUdpBuffer();
}

TEST(Robot, ThrowsIfConflictingOperationIsRunning) {
  std::atomic_bool run(true);
  std::atomic_bool started_sending(false);

  RobotMockServer server;
  Robot robot("127.0.0.1", RealtimeConfig::kIgnore);

  server
      .doForever(
          [&]() {
            bool continue_sending = run.load();
            if (continue_sending) {
              server.onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
                robot_state.motion_generator_mode = robot::MotionGeneratorMode::kIdle;
                robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
                robot_state.robot_mode = robot::RobotMode::kIdle;
              });
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
    robot.read([&](const RobotState&) {
      if (!running) {
        running = true;
        EXPECT_THROW(robot.read(std::function<bool(const RobotState&)>()),
                     InvalidOperationException);
      }
      std::this_thread::yield();
      return run.load();
    });
  });

  while (!running) {
    std::this_thread::yield();
  }

  EXPECT_THROW(robot.control(std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<JointPositions(const RobotState&, Duration)>(),
                             std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<JointVelocities(const RobotState&, Duration)>(),
                             std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<CartesianPose(const RobotState&, Duration)>(),
                             std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<CartesianVelocities(const RobotState&, Duration)>(),
                             std::function<Torques(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<JointPositions(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<JointVelocities(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<CartesianPose(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.control(std::function<CartesianVelocities(const RobotState&, Duration)>()),
               InvalidOperationException);
  EXPECT_THROW(robot.read(std::function<bool(const RobotState&)>()), InvalidOperationException);
  EXPECT_THROW(robot.readOnce(), InvalidOperationException);

  server.ignoreUdpBuffer();
  run = false;

  if (thread.joinable()) {
    thread.join();
  }
}
