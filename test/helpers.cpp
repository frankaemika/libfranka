#include "helpers.h"

#include <cstdlib>

#include <gtest/gtest.h>

void testRobotStateIsZero(const franka::RobotState& actual) {
  for (size_t i = 0; i < actual.O_T_EE.size(); i++) {
    EXPECT_EQ(0.0, actual.O_T_EE[i]);
  }
  for (size_t i = 0; i < actual.O_T_EE_d.size(); i++) {
    EXPECT_EQ(0.0, actual.O_T_EE_d[i]);
  }
  for (size_t i = 0; i < actual.elbow.size(); i++) {
    EXPECT_EQ(0.0, actual.elbow[i]);
  }
  for (size_t i = 0; i < actual.elbow_d.size(); i++) {
    EXPECT_EQ(0.0, actual.elbow_d[i]);
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
  for (size_t i = 0; i < actual.O_F_ext_hat_K.size(); i++) {
    EXPECT_EQ(0.0, actual.O_F_ext_hat_K[i]);
  }
  for (size_t i = 0; i < actual.K_F_ext_hat_K.size(); i++) {
    EXPECT_EQ(0.0, actual.K_F_ext_hat_K[i]);
  }
  EXPECT_EQ(0u, actual.sequence_number);
}

void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual) {
  EXPECT_EQ(expected.O_T_EE, actual.O_T_EE);
  EXPECT_EQ(expected.O_T_EE_d, actual.O_T_EE_d);
  EXPECT_EQ(expected.elbow, actual.elbow);
  EXPECT_EQ(expected.elbow_d, actual.elbow_d);
  EXPECT_EQ(expected.tau_J, actual.tau_J);
  EXPECT_EQ(expected.dtau_J, actual.dtau_J);
  EXPECT_EQ(expected.q, actual.q);
  EXPECT_EQ(expected.dq, actual.dq);
  EXPECT_EQ(expected.q_d, actual.q_d);
  EXPECT_EQ(expected.joint_contact, actual.joint_contact);
  EXPECT_EQ(expected.cartesian_contact, actual.cartesian_contact);
  EXPECT_EQ(expected.joint_collision, actual.joint_collision);
  EXPECT_EQ(expected.cartesian_collision, actual.cartesian_collision);
  EXPECT_EQ(expected.tau_ext_hat_filtered, actual.tau_ext_hat_filtered);
  EXPECT_EQ(expected.O_F_ext_hat_K, actual.O_F_ext_hat_K);
  EXPECT_EQ(expected.K_F_ext_hat_K, actual.K_F_ext_hat_K);
  EXPECT_EQ(expected.sequence_number, actual.sequence_number);
}

void testRobotStatesAreEqual(const research_interface::robot::RobotState& expected,
                             const franka::RobotState& actual) {
  EXPECT_EQ(expected.O_T_EE, actual.O_T_EE);
  EXPECT_EQ(expected.O_T_EE_d, actual.O_T_EE_d);
  EXPECT_EQ(expected.elbow, actual.elbow);
  EXPECT_EQ(expected.elbow_d, actual.elbow_d);
  EXPECT_EQ(expected.tau_J, actual.tau_J);
  EXPECT_EQ(expected.dtau_J, actual.dtau_J);
  EXPECT_EQ(expected.q, actual.q);
  EXPECT_EQ(expected.dq, actual.dq);
  EXPECT_EQ(expected.q_d, actual.q_d);
  EXPECT_EQ(expected.joint_contact, actual.joint_contact);
  EXPECT_EQ(expected.cartesian_contact, actual.cartesian_contact);
  EXPECT_EQ(expected.joint_collision, actual.joint_collision);
  EXPECT_EQ(expected.cartesian_collision, actual.cartesian_collision);
  EXPECT_EQ(expected.tau_ext_hat_filtered, actual.tau_ext_hat_filtered);
  EXPECT_EQ(expected.O_F_ext_hat_K, actual.O_F_ext_hat_K);
  EXPECT_EQ(expected.K_F_ext_hat_K, actual.K_F_ext_hat_K);
  EXPECT_EQ(expected.message_id, actual.sequence_number);
}

double randomDouble() {
  return 10.0 * static_cast<double>(std::rand()) / RAND_MAX;
}

bool randomBool() {
  return static_cast<bool>(std::rand() % 2);
}

void randomRobotState(franka::RobotState& robot_state) {
  for (size_t i = 0; i < robot_state.O_T_EE.size(); i++) {
    robot_state.O_T_EE[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.O_T_EE_d.size(); i++) {
    robot_state.O_T_EE_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.elbow.size(); i++) {
    robot_state.elbow[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.elbow_d.size(); i++) {
    robot_state.elbow_d[i] = randomDouble();
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
  for (size_t i = 0; i < robot_state.O_F_ext_hat_K.size(); i++) {
    robot_state.O_F_ext_hat_K[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.K_F_ext_hat_K.size(); i++) {
    robot_state.K_F_ext_hat_K[i] = randomDouble();
  }
  robot_state.sequence_number = static_cast<uint32_t>(std::rand());
}

void randomRobotState(research_interface::robot::RobotState& robot_state) {
  // Reset to all-zeros first
  robot_state = research_interface::robot::RobotState();
  for (size_t i = 0; i < robot_state.O_T_EE.size(); i++) {
    robot_state.O_T_EE[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.O_T_EE_d.size(); i++) {
    robot_state.O_T_EE_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.elbow.size(); i++) {
    robot_state.elbow[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.elbow_d.size(); i++) {
    robot_state.elbow_d[i] = randomDouble();
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
  for (size_t i = 0; i < robot_state.O_F_ext_hat_K.size(); i++) {
    robot_state.O_F_ext_hat_K[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_state.K_F_ext_hat_K.size(); i++) {
    robot_state.K_F_ext_hat_K[i] = randomDouble();
  }
  robot_state.message_id = static_cast<uint32_t>(std::rand());
  robot_state.motion_generator_mode = research_interface::robot::MotionGeneratorMode::kIdle;
  robot_state.controller_mode = research_interface::robot::ControllerMode::kMotorPD;
}

void randomRobotCommand(research_interface::robot::RobotCommand& robot_command) {
  // Reset to all-zeros first
  robot_command = research_interface::robot::RobotCommand();
  for (size_t i = 0; i < robot_command.motion.q_d.size(); i++) {
    robot_command.motion.q_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_command.motion.dq_d.size(); i++) {
    robot_command.motion.dq_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_command.motion.ddq_d.size(); i++) {
    robot_command.motion.ddq_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_command.motion.O_T_EE_d.size(); i++) {
    robot_command.motion.O_T_EE_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_command.motion.O_dP_EE_d.size(); i++) {
    robot_command.motion.O_dP_EE_d[i] = randomDouble();
  }
  for (size_t i = 0; i < robot_command.motion.elbow_d.size(); i++) {
    robot_command.motion.elbow_d[i] = randomDouble();
  }
  robot_command.motion.valid_elbow = true;
  robot_command.motion.motion_generation_finished = true;
  for (size_t i = 0; i < robot_command.control.tau_J_d.size(); i++) {
    robot_command.control.tau_J_d[i] = randomDouble();
  }
  robot_command.message_id = static_cast<uint32_t>(std::rand());
}

void testMotionGeneratorCommandsAreEqual(
    const research_interface::robot::MotionGeneratorCommand& expected,
    const research_interface::robot::MotionGeneratorCommand& actual) {
  EXPECT_EQ(expected.q_d, actual.q_d);
  EXPECT_EQ(expected.dq_d, actual.dq_d);
  EXPECT_EQ(expected.ddq_d, actual.ddq_d);
  EXPECT_EQ(expected.O_T_EE_d, actual.O_T_EE_d);
  EXPECT_EQ(expected.O_dP_EE_d, actual.O_dP_EE_d);
  EXPECT_EQ(expected.elbow_d, actual.elbow_d);
  EXPECT_EQ(expected.valid_elbow, actual.valid_elbow);
  EXPECT_EQ(expected.motion_generation_finished, actual.motion_generation_finished);
}

void testControllerCommandsAreEqual(const research_interface::robot::ControllerCommand& expected,
                                    const research_interface::robot::ControllerCommand& actual) {
  EXPECT_EQ(expected.tau_J_d, actual.tau_J_d);
}

void randomGripperState(franka::GripperState& gripper_state) {
  gripper_state.sequence_number = static_cast<uint32_t>(std::rand());
  gripper_state.temperature = static_cast<uint16_t>(std::rand());
  gripper_state.object_grasped = randomBool();
  gripper_state.max_opening_width = randomDouble();
  gripper_state.opening_width = randomDouble();
}

void randomGripperState(research_interface::gripper::GripperState& gripper_state) {
  // Reset to all-zeros first
  gripper_state = research_interface::gripper::GripperState();
  gripper_state.message_id = static_cast<uint32_t>(std::rand());
  gripper_state.temperature = static_cast<uint16_t>(std::rand());
  gripper_state.is_grasped = randomBool();
  gripper_state.max_width = randomDouble();
  gripper_state.width = randomDouble();
}

void testGripperStatesAreEqual(const franka::GripperState& expected,
                               const franka::GripperState& actual) {
  EXPECT_EQ(expected.sequence_number, actual.sequence_number);
  EXPECT_EQ(expected.opening_width, actual.opening_width);
  EXPECT_EQ(expected.max_opening_width, actual.max_opening_width);
  EXPECT_EQ(expected.object_grasped, actual.object_grasped);
  EXPECT_EQ(expected.temperature, actual.temperature);
}

void testGripperStatesAreEqual(const research_interface::gripper::GripperState& expected,
                               const franka::GripperState& actual) {
  EXPECT_EQ(expected.message_id, actual.sequence_number);
  EXPECT_EQ(expected.width, actual.opening_width);
  EXPECT_EQ(expected.max_width, actual.max_opening_width);
  EXPECT_EQ(expected.is_grasped, actual.object_grasped);
  EXPECT_EQ(expected.temperature, actual.temperature);
}

namespace research_interface {
namespace robot {

bool operator==(const Move::Deviation& left, const Move::Deviation& right) {
  return left.translation == right.translation && left.rotation == right.rotation &&
         left.elbow == right.elbow;
}

}  // namespace robot
}  // namespace research_interface
