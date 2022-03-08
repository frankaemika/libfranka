// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "helpers.h"

#include <cstdlib>

#include <gtest/gtest.h>

#include "load_calculations.h"

bool stringContains(const std::string& actual, const std::string& expected) {
  return actual.find(expected) != std::string::npos;
}

void testRobotStateIsZero(const franka::RobotState& actual) {
  for (double element : actual.O_T_EE) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.F_T_NE) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.NE_T_EE) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_T_EE_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.F_T_EE) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.EE_T_K) {
    EXPECT_EQ(0.0, element);
  }
  EXPECT_EQ(0.0, actual.m_ee);
  for (double element : actual.I_ee) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.F_x_Cee) {
    EXPECT_EQ(0.0, element);
  }
  EXPECT_EQ(0.0, actual.m_load);
  for (double element : actual.I_load) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.F_x_Cload) {
    EXPECT_EQ(0.0, element);
  }
  EXPECT_EQ(0.0, actual.m_total);
  for (double element : actual.I_total) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.F_x_Ctotal) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.elbow) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.elbow_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.elbow_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.delbow_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.ddelbow_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.tau_J) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.tau_J_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.dtau_J) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.q) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.q_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.dq) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.dq_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.ddq_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.joint_contact) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.cartesian_contact) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.joint_collision) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.cartesian_collision) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.tau_ext_hat_filtered) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_F_ext_hat_K) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.K_F_ext_hat_K) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_dP_EE_d) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_T_EE_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_dP_EE_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.O_ddP_EE_c) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.theta) {
    EXPECT_EQ(0.0, element);
  }
  for (double element : actual.dtheta) {
    EXPECT_EQ(0.0, element);
  }
  EXPECT_FALSE(actual.current_errors);
  EXPECT_FALSE(actual.last_motion_errors);
  EXPECT_EQ(0u, actual.time.toMSec());
  EXPECT_EQ(0.0, actual.control_command_success_rate);
}

void testRobotStatesAreEqual(const franka::RobotState& expected, const franka::RobotState& actual) {
  EXPECT_EQ(expected.O_T_EE, actual.O_T_EE);
  EXPECT_EQ(expected.F_T_NE, actual.F_T_NE);
  EXPECT_EQ(expected.NE_T_EE, actual.NE_T_EE);
  EXPECT_EQ(expected.O_T_EE_d, actual.O_T_EE_d);
  EXPECT_EQ(expected.F_T_EE, actual.F_T_EE);
  EXPECT_EQ(expected.EE_T_K, actual.EE_T_K);
  EXPECT_EQ(expected.m_ee, actual.m_ee);
  EXPECT_EQ(expected.F_x_Cee, actual.F_x_Cee);
  EXPECT_EQ(expected.I_ee, actual.I_ee);
  EXPECT_EQ(expected.m_load, actual.m_load);
  EXPECT_EQ(expected.F_x_Cload, actual.F_x_Cload);
  EXPECT_EQ(expected.I_load, actual.I_load);
  EXPECT_EQ(expected.m_total, actual.m_total);
  EXPECT_EQ(expected.F_x_Ctotal, actual.F_x_Ctotal);
  EXPECT_EQ(expected.I_total, actual.I_total);
  EXPECT_EQ(expected.elbow, actual.elbow);
  EXPECT_EQ(expected.elbow_d, actual.elbow_d);
  EXPECT_EQ(expected.elbow_c, actual.elbow_c);
  EXPECT_EQ(expected.delbow_c, actual.delbow_c);
  EXPECT_EQ(expected.ddelbow_c, actual.ddelbow_c);
  EXPECT_EQ(expected.tau_J, actual.tau_J);
  EXPECT_EQ(expected.tau_J_d, actual.tau_J_d);
  EXPECT_EQ(expected.dtau_J, actual.dtau_J);
  EXPECT_EQ(expected.q, actual.q);
  EXPECT_EQ(expected.dq, actual.dq);
  EXPECT_EQ(expected.q_d, actual.q_d);
  EXPECT_EQ(expected.dq_d, actual.dq_d);
  EXPECT_EQ(expected.ddq_d, actual.ddq_d);
  EXPECT_EQ(expected.joint_contact, actual.joint_contact);
  EXPECT_EQ(expected.cartesian_contact, actual.cartesian_contact);
  EXPECT_EQ(expected.joint_collision, actual.joint_collision);
  EXPECT_EQ(expected.cartesian_collision, actual.cartesian_collision);
  EXPECT_EQ(expected.tau_ext_hat_filtered, actual.tau_ext_hat_filtered);
  EXPECT_EQ(expected.O_F_ext_hat_K, actual.O_F_ext_hat_K);
  EXPECT_EQ(expected.K_F_ext_hat_K, actual.K_F_ext_hat_K);
  EXPECT_EQ(expected.O_dP_EE_d, actual.O_dP_EE_d);
  EXPECT_EQ(expected.O_T_EE_c, actual.O_T_EE_c);
  EXPECT_EQ(expected.O_dP_EE_c, actual.O_dP_EE_c);
  EXPECT_EQ(expected.O_ddP_EE_c, actual.O_ddP_EE_c);
  EXPECT_EQ(expected.theta, actual.theta);
  EXPECT_EQ(expected.dtheta, actual.dtheta);
  EXPECT_EQ(expected.current_errors, actual.current_errors);
  EXPECT_EQ(expected.last_motion_errors, actual.last_motion_errors);
  EXPECT_EQ(expected.control_command_success_rate, actual.control_command_success_rate);
  EXPECT_EQ(expected.robot_mode, actual.robot_mode);
  EXPECT_EQ(expected.time, actual.time);
}

void testRobotStatesAreEqual(const research_interface::robot::RobotState& expected,
                             const franka::RobotState& actual) {
  EXPECT_EQ(expected.O_T_EE, actual.O_T_EE);
  EXPECT_EQ(expected.F_T_NE, actual.F_T_NE);
  EXPECT_EQ(expected.NE_T_EE, actual.NE_T_EE);
  EXPECT_EQ(expected.O_T_EE_d, actual.O_T_EE_d);
  EXPECT_EQ(expected.F_T_EE, actual.F_T_EE);
  EXPECT_EQ(expected.EE_T_K, actual.EE_T_K);
  EXPECT_EQ(expected.m_ee, actual.m_ee);
  EXPECT_EQ(expected.F_x_Cee, actual.F_x_Cee);
  EXPECT_EQ(expected.I_ee, actual.I_ee);
  EXPECT_EQ(expected.m_load, actual.m_load);
  EXPECT_EQ(expected.F_x_Cload, actual.F_x_Cload);
  EXPECT_EQ(expected.I_load, actual.I_load);
  EXPECT_EQ(expected.m_ee + expected.m_load, actual.m_total);
  EXPECT_EQ(franka::combineCenterOfMass(expected.m_ee, expected.F_x_Cee, expected.m_load,
                                        expected.F_x_Cload),
            actual.F_x_Ctotal);
  EXPECT_EQ(franka::combineInertiaTensor(expected.m_ee, expected.F_x_Cee, expected.I_ee,
                                         expected.m_load, expected.F_x_Cload, expected.I_load,
                                         actual.m_total, actual.F_x_Ctotal),
            actual.I_total);
  EXPECT_EQ(expected.elbow, actual.elbow);
  EXPECT_EQ(expected.elbow_d, actual.elbow_d);
  EXPECT_EQ(expected.tau_J, actual.tau_J);
  EXPECT_EQ(expected.tau_J_d, actual.tau_J_d);
  EXPECT_EQ(expected.dtau_J, actual.dtau_J);
  EXPECT_EQ(expected.q, actual.q);
  EXPECT_EQ(expected.dq, actual.dq);
  EXPECT_EQ(expected.q_d, actual.q_d);
  EXPECT_EQ(expected.dq_d, actual.dq_d);
  EXPECT_EQ(expected.ddq_d, actual.ddq_d);
  EXPECT_EQ(expected.joint_contact, actual.joint_contact);
  EXPECT_EQ(expected.cartesian_contact, actual.cartesian_contact);
  EXPECT_EQ(expected.joint_collision, actual.joint_collision);
  EXPECT_EQ(expected.cartesian_collision, actual.cartesian_collision);
  EXPECT_EQ(expected.tau_ext_hat_filtered, actual.tau_ext_hat_filtered);
  EXPECT_EQ(expected.O_F_ext_hat_K, actual.O_F_ext_hat_K);
  EXPECT_EQ(expected.K_F_ext_hat_K, actual.K_F_ext_hat_K);
  EXPECT_EQ(expected.O_dP_EE_d, actual.O_dP_EE_d);
  EXPECT_EQ(expected.elbow_c, actual.elbow_c);
  EXPECT_EQ(expected.delbow_c, actual.delbow_c);
  EXPECT_EQ(expected.ddelbow_c, actual.ddelbow_c);
  EXPECT_EQ(expected.O_T_EE_c, actual.O_T_EE_c);
  EXPECT_EQ(expected.O_dP_EE_c, actual.O_dP_EE_c);
  EXPECT_EQ(expected.O_ddP_EE_c, actual.O_ddP_EE_c);
  EXPECT_EQ(expected.theta, actual.theta);
  EXPECT_EQ(expected.dtheta, actual.dtheta);
  EXPECT_EQ(franka::Errors(expected.errors), actual.current_errors);
  EXPECT_EQ(franka::Errors(expected.reflex_reason), actual.last_motion_errors);
  EXPECT_EQ(expected.message_id, actual.time.toMSec());
  EXPECT_EQ(expected.control_command_success_rate, actual.control_command_success_rate);

  franka::RobotMode expected_robot_mode;
  switch (expected.robot_mode) {
    case research_interface::robot::RobotMode::kOther:
      expected_robot_mode = franka::RobotMode::kOther;
      break;
    case research_interface::robot::RobotMode::kIdle:
      expected_robot_mode = franka::RobotMode::kIdle;
      break;
    case research_interface::robot::RobotMode::kMove:
      expected_robot_mode = franka::RobotMode::kMove;
      break;
    case research_interface::robot::RobotMode::kGuiding:
      expected_robot_mode = franka::RobotMode::kGuiding;
      break;
    case research_interface::robot::RobotMode::kReflex:
      expected_robot_mode = franka::RobotMode::kReflex;
      break;
    case research_interface::robot::RobotMode::kUserStopped:
      expected_robot_mode = franka::RobotMode::kUserStopped;
      break;
    case research_interface::robot::RobotMode::kAutomaticErrorRecovery:
      expected_robot_mode = franka::RobotMode::kAutomaticErrorRecovery;
      break;
  }
  EXPECT_EQ(expected_robot_mode, actual.robot_mode);
}

double randomDouble() {
  return 10.0 * static_cast<double>(std::rand()) / RAND_MAX;
}

bool randomBool() {
  return static_cast<bool>(std::rand() % 2);
}

std::array<double, 16> identityMatrix() {
  std::array<double, 16> matrix{};
  for (size_t j = 0; j < matrix.size(); j++) {
    if (j % 5 == 0) {
      matrix[j] = 1.0;
    }
  }
  return matrix;
}

franka::RobotState generateValidRobotState() {
  franka::RobotState robot_state{};
  robot_state.O_T_EE = identityMatrix();
  robot_state.O_T_EE_d = identityMatrix();
  robot_state.F_T_EE = identityMatrix();
  robot_state.EE_T_K = identityMatrix();
  robot_state.O_T_EE_c = identityMatrix();
  return robot_state;
}

void randomRobotState(franka::RobotState& robot_state) {
  for (double& element : robot_state.O_T_EE) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_T_NE) {
    element = randomDouble();
  }
  for (double& element : robot_state.NE_T_EE) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_T_EE_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_T_EE) {
    element = randomDouble();
  }
  for (double& element : robot_state.EE_T_K) {
    element = randomDouble();
  }
  robot_state.m_ee = randomDouble();
  for (double& element : robot_state.I_ee) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_x_Cee) {
    element = randomDouble();
  }
  robot_state.m_load = randomDouble();
  for (double& element : robot_state.I_load) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_x_Cload) {
    element = randomDouble();
  }
  robot_state.m_total = randomDouble();
  for (double& element : robot_state.I_total) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_x_Ctotal) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.delbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.ddelbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.tau_J) {
    element = randomDouble();
  }
  for (double& element : robot_state.tau_J_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.dtau_J) {
    element = randomDouble();
  }
  for (double& element : robot_state.q) {
    element = randomDouble();
  }
  for (double& element : robot_state.q_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.dq) {
    element = randomDouble();
  }
  for (double& element : robot_state.dq_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.ddq_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.joint_contact) {
    element = randomDouble();
  }
  for (double& element : robot_state.cartesian_contact) {
    element = randomDouble();
  }
  for (double& element : robot_state.joint_collision) {
    element = randomDouble();
  }
  for (double& element : robot_state.cartesian_collision) {
    element = randomDouble();
  }
  for (double& element : robot_state.tau_ext_hat_filtered) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_F_ext_hat_K) {
    element = randomDouble();
  }
  for (double& element : robot_state.K_F_ext_hat_K) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_dP_EE_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_T_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_dP_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_ddP_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.theta) {
    element = randomDouble();
  }
  for (double& element : robot_state.dtheta) {
    element = randomDouble();
  }
  std::array<bool, sizeof(research_interface::robot::RobotState::errors)> errors{};
  for (bool& error : errors) {
    error = randomBool();
  }
  robot_state.current_errors = franka::Errors(errors);
  for (bool& error : errors) {
    error = randomBool();
  }
  robot_state.last_motion_errors = franka::Errors(errors);
  robot_state.control_command_success_rate = randomDouble();
  robot_state.time = franka::Duration(static_cast<uint64_t>(std::rand()));
}

void randomRobotState(research_interface::robot::RobotState& robot_state) {
  // Reset to all-zeros first
  robot_state = research_interface::robot::RobotState();
  for (double& element : robot_state.O_T_EE) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_T_NE) {
    element = randomDouble();
  }
  for (double& element : robot_state.NE_T_EE) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_T_EE_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.EE_T_K) {
    element = randomDouble();
  }
  for (double& element : robot_state.F_T_EE) {
    element = randomDouble();
  }
  robot_state.m_ee = randomDouble();
  for (double& element : robot_state.F_x_Cee) {
    element = randomDouble();
  }
  for (double& element : robot_state.I_ee) {
    element = randomDouble();
  }
  robot_state.m_load = randomDouble();
  for (double& element : robot_state.F_x_Cload) {
    element = randomDouble();
  }
  for (double& element : robot_state.I_load) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.tau_J) {
    element = randomDouble();
  }
  for (double& element : robot_state.dtau_J) {
    element = randomDouble();
  }
  for (double& element : robot_state.q) {
    element = randomDouble();
  }
  for (double& element : robot_state.dq) {
    element = randomDouble();
  }
  for (double& element : robot_state.q_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.dq_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.ddq_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.joint_contact) {
    element = randomDouble();
  }
  for (double& element : robot_state.cartesian_contact) {
    element = randomDouble();
  }
  for (double& element : robot_state.joint_collision) {
    element = randomDouble();
  }
  for (double& element : robot_state.cartesian_collision) {
    element = randomDouble();
  }
  for (double& element : robot_state.tau_ext_hat_filtered) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_F_ext_hat_K) {
    element = randomDouble();
  }
  for (double& element : robot_state.K_F_ext_hat_K) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_dP_EE_d) {
    element = randomDouble();
  }
  for (double& element : robot_state.elbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.delbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.ddelbow_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_T_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_dP_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.O_ddP_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_state.theta) {
    element = randomDouble();
  }
  for (double& element : robot_state.dtheta) {
    element = randomDouble();
  }
  for (bool& error : robot_state.errors) {
    error = randomBool();
  }
  for (bool& element : robot_state.reflex_reason) {
    element = randomBool();
  }
  robot_state.message_id = static_cast<uint32_t>(std::rand());
  robot_state.control_command_success_rate = randomDouble();
  robot_state.motion_generator_mode = research_interface::robot::MotionGeneratorMode::kIdle;
  robot_state.controller_mode = research_interface::robot::ControllerMode::kJointImpedance;
}

void randomRobotCommand(research_interface::robot::RobotCommand& robot_command) {
  // Reset to all-zeros first
  robot_command = research_interface::robot::RobotCommand();
  for (double& element : robot_command.motion.q_c) {
    element = randomDouble();
  }
  for (double& element : robot_command.motion.dq_c) {
    element = randomDouble();
  }
  for (double& element : robot_command.motion.O_T_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_command.motion.O_dP_EE_c) {
    element = randomDouble();
  }
  for (double& element : robot_command.motion.elbow_c) {
    element = randomDouble();
  }
  robot_command.motion.valid_elbow = true;
  robot_command.motion.motion_generation_finished = true;
  for (double& element : robot_command.control.tau_J_d) {
    element = randomDouble();
  }
  robot_command.message_id = static_cast<uint32_t>(std::rand());
}

void testMotionGeneratorCommandsAreEqual(
    const research_interface::robot::MotionGeneratorCommand& expected,
    const research_interface::robot::MotionGeneratorCommand& actual) {
  EXPECT_EQ(expected.q_c, actual.q_c);
  EXPECT_EQ(expected.dq_c, actual.dq_c);
  EXPECT_EQ(expected.O_T_EE_c, actual.O_T_EE_c);
  EXPECT_EQ(expected.O_dP_EE_c, actual.O_dP_EE_c);
  EXPECT_EQ(expected.elbow_c, actual.elbow_c);
  EXPECT_EQ(expected.valid_elbow, actual.valid_elbow);
  EXPECT_EQ(expected.motion_generation_finished, actual.motion_generation_finished);
}

void testControllerCommandsAreEqual(const research_interface::robot::ControllerCommand& expected,
                                    const research_interface::robot::ControllerCommand& actual) {
  EXPECT_EQ(expected.tau_J_d, actual.tau_J_d);
}

void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const research_interface::robot::RobotCommand& actual) {
  testMotionGeneratorCommandsAreEqual(expected.motion, actual.motion);
  testControllerCommandsAreEqual(expected.control, actual.control);
  EXPECT_EQ(expected.message_id, actual.message_id);
}

void testRobotCommandsAreEqual(const research_interface::robot::RobotCommand& expected,
                               const franka::RobotCommand actual) {
  research_interface::robot::RobotCommand fci_command = expected;
  fci_command.motion.q_c = actual.joint_positions.q;
  fci_command.motion.dq_c = actual.joint_velocities.dq;
  fci_command.motion.O_T_EE_c = actual.cartesian_pose.O_T_EE;
  fci_command.motion.O_dP_EE_c = actual.cartesian_velocities.O_dP_EE;
  fci_command.control.tau_J_d = actual.torques.tau_J;
  testRobotCommandsAreEqual(expected, fci_command);
}

void randomGripperState(franka::GripperState& gripper_state) {
  gripper_state.time = franka::Duration(static_cast<uint64_t>(std::rand()));
  gripper_state.temperature = static_cast<uint16_t>(std::rand());
  gripper_state.is_grasped = randomBool();
  gripper_state.max_width = randomDouble();
  gripper_state.width = randomDouble();
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
  EXPECT_EQ(expected.time, actual.time);
  EXPECT_EQ(expected.width, actual.width);
  EXPECT_EQ(expected.max_width, actual.max_width);
  EXPECT_EQ(expected.is_grasped, actual.is_grasped);
  EXPECT_EQ(expected.temperature, actual.temperature);
}

void testGripperStatesAreEqual(const research_interface::gripper::GripperState& expected,
                               const franka::GripperState& actual) {
  EXPECT_EQ(expected.message_id, actual.time.toMSec());
  EXPECT_EQ(expected.width, actual.width);
  EXPECT_EQ(expected.max_width, actual.max_width);
  EXPECT_EQ(expected.is_grasped, actual.is_grasped);
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

namespace franka {

bool operator==(const Errors& lhs, const Errors& rhs) {
  return lhs.joint_position_limits_violation == rhs.joint_position_limits_violation &&
         lhs.cartesian_position_limits_violation == rhs.cartesian_position_limits_violation &&
         lhs.self_collision_avoidance_violation == rhs.self_collision_avoidance_violation &&
         lhs.joint_velocity_violation == rhs.joint_velocity_violation &&
         lhs.cartesian_velocity_violation == rhs.cartesian_velocity_violation &&
         lhs.force_control_safety_violation == rhs.force_control_safety_violation &&
         lhs.joint_reflex == rhs.joint_reflex && lhs.cartesian_reflex == rhs.cartesian_reflex &&
         lhs.max_goal_pose_deviation_violation == rhs.max_goal_pose_deviation_violation &&
         lhs.max_path_pose_deviation_violation == rhs.max_path_pose_deviation_violation &&
         lhs.cartesian_velocity_profile_safety_violation ==
             rhs.cartesian_velocity_profile_safety_violation &&
         lhs.joint_position_motion_generator_start_pose_invalid ==
             rhs.joint_position_motion_generator_start_pose_invalid &&
         lhs.joint_motion_generator_position_limits_violation ==
             rhs.joint_motion_generator_position_limits_violation &&
         lhs.joint_motion_generator_velocity_limits_violation ==
             rhs.joint_motion_generator_velocity_limits_violation &&
         lhs.joint_motion_generator_velocity_discontinuity ==
             rhs.joint_motion_generator_velocity_discontinuity &&
         lhs.joint_motion_generator_acceleration_discontinuity ==
             rhs.joint_motion_generator_acceleration_discontinuity &&
         lhs.cartesian_position_motion_generator_start_pose_invalid ==
             rhs.cartesian_position_motion_generator_start_pose_invalid &&
         lhs.cartesian_motion_generator_elbow_limit_violation ==
             rhs.cartesian_motion_generator_elbow_limit_violation &&
         lhs.cartesian_motion_generator_velocity_limits_violation ==
             rhs.cartesian_motion_generator_velocity_limits_violation &&
         lhs.cartesian_motion_generator_velocity_discontinuity ==
             rhs.cartesian_motion_generator_velocity_discontinuity &&
         lhs.cartesian_motion_generator_acceleration_discontinuity ==
             rhs.cartesian_motion_generator_acceleration_discontinuity &&
         lhs.cartesian_motion_generator_elbow_sign_inconsistent ==
             rhs.cartesian_motion_generator_elbow_sign_inconsistent &&
         lhs.cartesian_motion_generator_start_elbow_invalid ==
             rhs.cartesian_motion_generator_start_elbow_invalid &&
         lhs.cartesian_motion_generator_joint_position_limits_violation ==
             rhs.cartesian_motion_generator_joint_position_limits_violation &&
         lhs.cartesian_motion_generator_joint_velocity_limits_violation ==
             rhs.cartesian_motion_generator_joint_velocity_limits_violation &&
         lhs.cartesian_motion_generator_joint_velocity_discontinuity ==
             rhs.cartesian_motion_generator_joint_velocity_discontinuity &&
         lhs.cartesian_motion_generator_joint_acceleration_discontinuity ==
             rhs.cartesian_motion_generator_joint_acceleration_discontinuity &&
         lhs.cartesian_position_motion_generator_invalid_frame ==
             rhs.cartesian_position_motion_generator_invalid_frame &&
         lhs.force_controller_desired_force_tolerance_violation ==
             rhs.force_controller_desired_force_tolerance_violation &&
         lhs.controller_torque_discontinuity == rhs.controller_torque_discontinuity &&
         lhs.start_elbow_sign_inconsistent == rhs.start_elbow_sign_inconsistent &&
         lhs.communication_constraints_violation == rhs.communication_constraints_violation &&
         lhs.power_limit_violation == rhs.power_limit_violation &&
         lhs.joint_p2p_insufficient_torque_for_planning ==
             rhs.joint_p2p_insufficient_torque_for_planning &&
         lhs.tau_j_range_violation == rhs.tau_j_range_violation &&
         lhs.instability_detected == rhs.instability_detected &&
         lhs.joint_move_in_wrong_direction == rhs.joint_move_in_wrong_direction &&
         lhs.cartesian_spline_motion_generator_violation ==
             rhs.cartesian_spline_motion_generator_violation &&
         lhs.joint_via_motion_generator_planning_joint_limit_violation ==
             rhs.joint_via_motion_generator_planning_joint_limit_violation &&
         lhs.base_acceleration_initialization_timeout ==
             rhs.base_acceleration_initialization_timeout &&
         lhs.base_acceleration_invalid_reading == rhs.base_acceleration_invalid_reading;
}
}  // namespace franka
