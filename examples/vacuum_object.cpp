// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/vacuum_gripper.h>

/**
 * @example vacuum_object.cpp
 * An example showing how to control FRANKA's vacuum gripper.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./vacuum_object <vacuum-gripper-hostname>" << std::endl;
    return -1;
  }

  franka::VacuumGripper vacuum_gripper(argv[1]);
  try {
    // Print a vacuum gripper state.
    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "Initial vacuum gripper state: " << vacuum_gripper_state << std::endl;

    // Vacuum the object.
    if (!vacuum_gripper.vacuum(100, std::chrono::milliseconds(1000))) {
      std::cout << "Failed to vacuum the object." << std::endl;
      return -1;
    }

    vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "Vacuum gripper state after applying vacuum: " << vacuum_gripper_state
              << std::endl;

    // Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    vacuum_gripper_state = vacuum_gripper.readOnce();
    if (!vacuum_gripper_state.in_control_range) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    std::cout << "Vacuumed object, will release it now." << std::endl;
    vacuum_gripper.dropOff(std::chrono::milliseconds(1000));
  } catch (franka::Exception const& e) {
    vacuum_gripper.stop();
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
