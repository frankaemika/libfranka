// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <sstream>
#include <string>
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

  try {
    franka::VacuumGripper vacuum_gripper(argv[1]);

    // Print a vacuum gripper state.
    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "Vacuum gripper state: " << vacuum_gripper_state << std::endl;

    // Vacuum the object.
    if (!vacuum_gripper.vacuum(100,0,std::chrono::milliseconds(1000))) {
      std::cout << "Failed to vacuum the object." << std::endl;
      return -1;
    }

    // Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    vacuum_gripper_state = vacuum_gripper.readOnce();
    if (!vacuum_gripper_state.vacuum) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    std::cout << "Vacuumed object, will release it now." << std::endl;
    vacuum_gripper.dropOff(std::chrono::milliseconds(1000));
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
