#include <iostream>
#include <sstream>
#include <thread>

#include <franka/gripper.h>
#include <franka/gripper_state.h>

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: ./grasp_object <gripper-hostname> <homing> <object-width>" << std::endl;
    return -1;
  }

  try {
    franka::Gripper gripper(argv[1]);
    double grasping_width = atof(argv[3]);

    std::stringstream ss(argv[2]);
    bool homing;
    if (!(ss >> homing)) {
      std::cerr << "<homing> can be 0 or 1." << std::endl;
      return -1;
    }

    if (homing) {
      // Do a homing in order to estimate the maximum grasping width with the current fingers.
      gripper.homing();
    }

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
    if (gripper_state.max_opening_width < grasping_width) {
      std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
      return -1;
    }

    // Grasp the object.
    if (!gripper.grasp(grasping_width, 0.1, 300)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }

    // Wait 2s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    gripper_state = gripper.readOnce();
    if (!gripper_state.object_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
