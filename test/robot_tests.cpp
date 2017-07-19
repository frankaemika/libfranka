#include <gmock/gmock.h>

#include <atomic>
#include <functional>

#include <franka/robot.h>

#include "helpers.h"
#include "mock_server.h"

using ::testing::_;
using ::testing::Return;

using research_interface::robot::Connect;
using research_interface::robot::Move;

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
  RobotMockServer server([](const Connect::Request&) {
    return Connect::Response(Connect::Status::kIncompatibleLibraryVersion);
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
  using namespace research_interface;

  RobotMockServer server;
  Robot robot("127.0.0.1");

  std::atomic_flag send = ATOMIC_FLAG_INIT;
  send.test_and_set();

  server
      .onSendUDP<robot::RobotState>([](robot::RobotState& robot_state) {
        robot_state.motion_generator_mode = robot::MotionGeneratorMode::kJointPosition;
        robot_state.controller_mode = robot::ControllerMode::kJointImpedance;
        robot_state.robot_mode = robot::RobotMode::kMove;
      })
      .spinOnce()
      .waitForCommand<Move>([&](const Move::Request&) {
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
            })
            .sendResponse<Move::Response>([]() { return Move::Response(Move::Status::kSuccess); });
        return Move::Response(Move::Status::kMotionStarted);
      })
      .spinOnce();

  int count = 0;
  robot.control(
      [&](const RobotState& robot_state) -> JointPositions {
        if (count == 0) {
          EXPECT_EQ(0u, robot_state.ticks);
          EXPECT_EQ(0.0, robot_state.time_step);
        } else {
          EXPECT_GE(robot_state.ticks, 1u);
          EXPECT_GE(robot_state.time_step, 0.001);
        }
        if (++count < 5) {
          return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        }
        send.clear();
        return Stop;
      },
      ControllerMode::kJointImpedance);

  for (int i = 0; i < count + 1; i++) {
    server.onReceiveRobotCommand().spinOnce();
  }

  EXPECT_EQ(5, count);
}
