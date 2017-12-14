// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example execute_trajectory.cpp
 * An example showing how to execute a joint trajectory loaded from a CSV file.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <trajectory-csv> <output>" << std::endl;
    return -1;
  }
  std::cout << "Loading csv trajectory" << std::endl;
  std::fstream csv_file_stream;
  csv_file_stream.open(argv[2], std::fstream::in);
  std::vector<std::array<double, 7>> samples;
  while (csv_file_stream) {
    std::array<double, 7> q;
    char delimiter;
    for (size_t i = 0; i < 7; i++) {
      csv_file_stream >> q[i] >> delimiter;
    }
    samples.push_back(q);
  }
  std::cout << "Read " << samples.size() << " samples" << std::endl;

  std::vector<franka::RobotState> states;
  try {
    franka::Robot robot(argv[1]);

    // First move the robot to a suitable joint configuration
    MotionGenerator motion_generator(0.5, samples[0]);
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

    const std::array<double, 7> max_joint_vel{{2.375, 2.375, 2.375, 2.375, 2.375, 2.375, 2.375}};
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    size_t index = 0;
    robot.control([&](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      states.push_back(robot_state);

      index += period.toMSec();

      if (index >= samples.size() - 1) {
        return franka::MotionFinished(franka::JointPositions(samples.back()));
      }
      // state.q_d contains the last joint position command received by the robot.
      // In case of packet loss due to bad connection or due to a slow control loop
      // not reaching the 1Khz rate, even if your desired trajectory
      // is smooth, discontinuities might occur.
      // Saturating the velocity computed with respect to the last command received
      // by the robot will prevent from getting discontinuity errors.
      // Note that if the robot does not receive a command it will try to extrapolate
      // the desired behavior assuming a constant acceleration model
      return limitRate(max_joint_vel, samples[index], robot_state.q_d);
    });
  } catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  std::cout << "Logging results to file: " << argv[3] << std::endl;
  std::fstream output_stream;
  output_stream.open(argv[3], std::fstream::out);
  for (size_t s = 0; s < states.size(); s++) {
    output_stream << "Sample: #" << s << std::endl;
    output_stream << "q_d: \t" << samples[s] << std::endl;
    output_stream << "Robot state:" << std::endl;
    output_stream << "q: \t" << states[s].q << std::endl;
    output_stream << "q_d: \t" << states[s].q_d << std::endl;
    output_stream << "dq: \t" << states[s].dq << std::endl;
    output_stream << "______" << std::endl;
  }

  return 0;
}
