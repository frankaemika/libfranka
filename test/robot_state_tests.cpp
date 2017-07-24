#include <gtest/gtest.h>

#include <sstream>

#include <franka/robot_state.h>
#include <robot_impl.h>

#include "helpers.h"

using namespace franka;

TEST(RobotState, IsInitializedToZero) {
  RobotState robot_state;

  testRobotStateIsZero(robot_state);
}

TEST(RobotState, CanBeStreamed) {
  RobotState robot_state;

  std::stringstream ss;
  ss << robot_state;
  std::string output(ss.str());

  EXPECT_PRED2(stringContains, output, "O_T_EE");
  EXPECT_PRED2(stringContains, output, "O_T_EE_d");
  EXPECT_PRED2(stringContains, output, "EE_T_K");
  EXPECT_PRED2(stringContains, output, "elbow");
  EXPECT_PRED2(stringContains, output, "elbow_d");
  EXPECT_PRED2(stringContains, output, "tau_J");
  EXPECT_PRED2(stringContains, output, "dtau_J");
  EXPECT_PRED2(stringContains, output, "q");
  EXPECT_PRED2(stringContains, output, "dq");
  EXPECT_PRED2(stringContains, output, "q_d");
  EXPECT_PRED2(stringContains, output, "joint_contact");
  EXPECT_PRED2(stringContains, output, "cartesian_contact");
  EXPECT_PRED2(stringContains, output, "joint_collision");
  EXPECT_PRED2(stringContains, output, "cartesian_collision");
  EXPECT_PRED2(stringContains, output, "tau_ext_hat_filtered");
  EXPECT_PRED2(stringContains, output, "O_F_ext_hat_K");
  EXPECT_PRED2(stringContains, output, "K_F_ext_hat_K");
  EXPECT_PRED2(stringContains, output, "current_errors");
  EXPECT_PRED2(stringContains, output, "last_motion_errors");
  EXPECT_PRED2(stringContains, output, "sequence_number");
}

TEST(RobotState, CanCopyConstruct) {
  RobotState robot_state;
  randomRobotState(robot_state);

  RobotState robot_state2(robot_state);

  testRobotStatesAreEqual(robot_state, robot_state2);
}

TEST(RobotState, CanAssign) {
  RobotState robot_state;
  randomRobotState(robot_state);

  RobotState robot_state2;
  robot_state2 = robot_state;

  testRobotStatesAreEqual(robot_state, robot_state2);
}

TEST(RobotState, CanConvert) {
  research_interface::robot::RobotState original;
  randomRobotState(original);

  RobotState converted = convertRobotState(original, original.message_id - 1);
  testRobotStatesAreEqual(original, converted);
  EXPECT_EQ(1u, converted.ticks);
}

TEST(RobotState, CanConvertWithOverflowingMessageId) {
  research_interface::robot::RobotState original{};
  original.message_id = 0;
  RobotState converted = convertRobotState(original, std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(1u, converted.ticks);

  original.message_id = 1;
  converted = convertRobotState(original, std::numeric_limits<uint32_t>::max() - 1);
  EXPECT_EQ(3u, converted.ticks);
}

TEST(RobotState, CanConvertWithMessageId) {
  research_interface::robot::RobotState original{};
  original.message_id = 1;
  RobotState converted = convertRobotState(original, 0);
  EXPECT_EQ(1u, converted.ticks);

  original.message_id = 12;
  converted = convertRobotState(original, 0);
  EXPECT_EQ(1u, converted.ticks);
}
