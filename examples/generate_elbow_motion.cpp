// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example generate_elbow_motion.cpp
 * An example showing how to move the robot's elbow.
 *
 * @warning Before executing this example, make sure that the elbow has enough space to move.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_elbow_motion <robot-hostname>" << std::endl;
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

    std::array<double, 16> initial_pose;
    std::array<double, 2> initial_elbow;
    double time = 0.0;
    robot.control([&time, &initial_pose, &initial_elbow](
                      const franka::RobotState& robot_state,
                      franka::Duration time_step) -> franka::CartesianPose {
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_d;
        initial_elbow = robot_state.elbow_d;
      }

      time += time_step.toSec();

      double angle = M_PI / 10.0 * (1.0 - std::cos(M_PI / 5.0 * time));

      auto elbow = initial_elbow;
      elbow[0] += angle;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished({initial_pose, elbow});
      }

      return {initial_pose, elbow};
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
