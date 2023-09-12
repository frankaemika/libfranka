// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/active_control.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  bool use_external_control_loop = false;
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> optional: <use_external_control_loop>"
              << std::endl;
    return -1;
  } else if (argc == 3) {
    use_external_control_loop = !std::strcmp(argv[2], "true");
  }

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    double time_max = 1.0;
    double omega_max = 1.0;
    double time = 0.0;

    auto callback_control = [=, &time](const franka::RobotState&,
                                       franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};

      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(velocities);
      }
      return velocities;
    };

    if (use_external_control_loop) {
      bool motion_finished = false;
      auto active_control = robot.startJointVelocityControl(
          research_interface::robot::Move::ControllerMode::kJointImpedance);
      while (!motion_finished) {
        auto [robot_state, duration] = active_control->readOnce();
        auto joint_velocities = callback_control(robot_state, duration);
        motion_finished = joint_velocities.motion_finished;
        active_control->writeOnce(joint_velocities);
      }
    } else {
      robot.control(callback_control);
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
