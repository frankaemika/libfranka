#include "helpers.h"

#include <cstdlib>

#include <gtest/gtest.h>

using namespace franka;

void testRobotStateIsZero(const RobotState& actual) {
  for (size_t i = 0; i < actual.q_start.size(); i++) {
    EXPECT_EQ(0.0, actual.q_start[i]);
  }
  for (size_t i = 0; i < actual.O_T_EE_start.size(); i++) {
    EXPECT_EQ(0.0, actual.O_T_EE_start[i]);
  }
  for (size_t i = 0; i < actual.elbow_start.size(); i++) {
    EXPECT_EQ(0.0, actual.elbow_start[i]);
  }
  for (size_t i = 0; i < actual.tau_J.size(); i++) {
    EXPECT_EQ(0.0, actual.tau_J[i]);
  }
  for (size_t i = 0; i < actual.dtau_J.size(); i++) {
    EXPECT_EQ(0.0, actual.dtau_J[i]);
  }
  for (size_t i = 0; i < actual.q.size(); i++) {
    EXPECT_EQ(0.0, actual.q[i]);
  }
  for (size_t i = 0; i < actual.dq.size(); i++) {
    EXPECT_EQ(0.0, actual.dq[i]);
  }
  for (size_t i = 0; i < actual.q_d.size(); i++) {
    EXPECT_EQ(0.0, actual.q_d[i]);
  }
  for (size_t i = 0; i < actual.joint_contact.size(); i++) {
    EXPECT_EQ(0.0, actual.joint_contact[i]);
  }
  for (size_t i = 0; i < actual.cartesian_contact.size(); i++) {
    EXPECT_EQ(0.0, actual.cartesian_contact[i]);
  }
  for (size_t i = 0; i < actual.joint_collision.size(); i++) {
    EXPECT_EQ(0.0, actual.joint_collision[i]);
  }
  for (size_t i = 0; i < actual.cartesian_collision.size(); i++) {
    EXPECT_EQ(0.0, actual.cartesian_collision[i]);
  }
  for (size_t i = 0; i < actual.tau_ext_hat_filtered.size(); i++) {
    EXPECT_EQ(0.0, actual.tau_ext_hat_filtered[i]);
  }
  for (size_t i = 0; i < actual.O_F_ext_hat_EE.size(); i++) {
    EXPECT_EQ(0.0, actual.O_F_ext_hat_EE[i]);
  }
  for (size_t i = 0; i < actual.EE_F_ext_hat_EE.size(); i++) {
    EXPECT_EQ(0.0, actual.EE_F_ext_hat_EE[i]);
  }
}

void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual) {
  for (size_t i = 0; i < actual.q_start.size(); i++) {
    EXPECT_EQ(expected.q_start[i], actual.q_start[i]);
  }
  for (size_t i = 0; i < actual.O_T_EE_start.size(); i++) {
    EXPECT_EQ(expected.O_T_EE_start[i], actual.O_T_EE_start[i]);
  }
  for (size_t i = 0; i < actual.elbow_start.size(); i++) {
    EXPECT_EQ(expected.elbow_start[i], actual.elbow_start[i]);
  }
  for (size_t i = 0; i < actual.tau_J.size(); i++) {
    EXPECT_EQ(expected.tau_J[i], actual.tau_J[i]);
  }
  for (size_t i = 0; i < actual.dtau_J.size(); i++) {
    EXPECT_EQ(expected.dtau_J[i], actual.dtau_J[i]);
  }
  for (size_t i = 0; i < actual.q.size(); i++) {
    EXPECT_EQ(expected.q[i], actual.q[i]);
  }
  for (size_t i = 0; i < actual.dq.size(); i++) {
    EXPECT_EQ(expected.dq[i], actual.dq[i]);
  }
  for (size_t i = 0; i < actual.q_d.size(); i++) {
    EXPECT_EQ(expected.q_d[i], actual.q_d[i]);
  }
  for (size_t i = 0; i < actual.joint_contact.size(); i++) {
    EXPECT_EQ(expected.joint_contact[i], actual.joint_contact[i]);
  }
  for (size_t i = 0; i < actual.cartesian_contact.size(); i++) {
    EXPECT_EQ(expected.cartesian_contact[i], actual.cartesian_contact[i]);
  }
  for (size_t i = 0; i < actual.joint_collision.size(); i++) {
    EXPECT_EQ(expected.joint_collision[i], actual.joint_collision[i]);
  }
  for (size_t i = 0; i < actual.cartesian_collision.size(); i++) {
    EXPECT_EQ(expected.cartesian_collision[i], actual.cartesian_collision[i]);
  }
  for (size_t i = 0; i < actual.tau_ext_hat_filtered.size(); i++) {
    EXPECT_EQ(expected.tau_ext_hat_filtered[i], actual.tau_ext_hat_filtered[i]);
  }
  for (size_t i = 0; i < actual.O_F_ext_hat_EE.size(); i++) {
    EXPECT_EQ(expected.O_F_ext_hat_EE[i], actual.O_F_ext_hat_EE[i]);
  }
  for (size_t i = 0; i < actual.EE_F_ext_hat_EE.size(); i++) {
    EXPECT_EQ(expected.EE_F_ext_hat_EE[i], actual.EE_F_ext_hat_EE[i]);
  }
}

double randomDouble() {
  return 10.0 * static_cast<double>(std::rand()) / RAND_MAX;
}

void randomRobotState(RobotState& robot_state) {
  for (size_t i = 0; i < robot_state.q_start.size(); i++) {
    robot_state.q_start[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.O_T_EE_start.size(); i++) {
    robot_state.O_T_EE_start[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.elbow_start.size(); i++) {
    robot_state.elbow_start[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.tau_J.size(); i++) {
    robot_state.tau_J[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.dtau_J.size(); i++) {
    robot_state.dtau_J[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.q.size(); i++) {
    robot_state.q[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.dq.size(); i++) {
    robot_state.dq[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.q_d.size(); i++) {
    robot_state.q_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.joint_contact.size(); i++) {
    robot_state.joint_contact[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.cartesian_contact.size(); i++) {
    robot_state.cartesian_contact[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.joint_collision.size(); i++) {
    robot_state.joint_collision[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.cartesian_collision.size(); i++) {
    robot_state.cartesian_collision[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.tau_ext_hat_filtered.size(); i++) {
    robot_state.tau_ext_hat_filtered[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.O_F_ext_hat_EE.size(); i++) {
    robot_state.O_F_ext_hat_EE[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.EE_F_ext_hat_EE.size(); i++) {
    robot_state.EE_F_ext_hat_EE[i] = randomDouble();
  }
}
