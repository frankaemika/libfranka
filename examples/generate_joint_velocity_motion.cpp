// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

std::array<double, 7> saturateDesiredJointAcceleration(const std::array<double, 7>& max_joint_acc,
                                                       const std::array<double, 7>& dq_d,
                                                       const std::array<double, 7>& last_dq_d);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_joint_velocity_motion <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    const std::array<double, 7>  max_joint_acc{{14.25, 7.125, 11.875, 11.875, 14.25, 19.0, 19.0}};

    double time_max = 1.0;
    double omega_max = 1.0;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState& state,
                             franka::Duration time_step) -> franka::JointVelocities {
      time += time_step.toSec();

      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};

      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(velocities);
      }
      // state.dq_d contains the last joint velocity command received by the robot.
      // In case of packet loss due to a bad connection, even if your desired trajectory
      // is smooth discontinuities might occur.
      // Saturating the acceleration computed with respect to the last command received
      // by the robot will prevent from getting discontinuity errors.
      return saturateDesiredJointAcceleration(max_joint_acc, velocities.dq, state.dq_d);
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

std::array<double, 7> saturateDesiredJointAcceleration(const std::array<double, 7>& max_joint_acc,
                                                       const std::array<double, 7>& dq_d,
                                                       const std::array<double, 7>& last_dq_d) {
  std::array<double, 7> dq_d_saturated{};
  for (size_t i = 1 ; i < 7 ; i ++) {
    double accel = (dq_d[i] - last_dq_d[i])/1e-3;
    dq_d_saturated[i] = last_dq_d[i] + std::max(std::min(accel, max_joint_acc[i]), -max_joint_acc[i])*1e-3;
  }
  return dq_d_saturated;
};
