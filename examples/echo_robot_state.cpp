#include <iostream>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-ip>" << std::endl;
    return -1;
  }

  std::string robot_ip(argv[1]);
  auto robot = franka::Robot::connect(robot_ip);

  while (robot->waitForRobotState()) {
    auto robotState = robot->getRobotState();
    std::cout << robotState << std::endl;
  }

  return 0;
}
