#include <franka/robot.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);  // NOLINT

    while (robot.waitForRobotState()) {
      const franka::RobotState& robot_state = robot.getRobotState();
      std::cout << robot_state << std::endl;
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
