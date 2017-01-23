#include <iostream>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-ip>" << std::endl;
    return -1;
  }

  std::string robot_ip(argv[1]);

  try
  {
    franka::Robot robot(robot_ip);

    while (robot.waitForRobotState()) {
      const franka::RobotState& robotState = robot.getRobotState();
      std::cout << robotState << std::endl;
    }
  }catch(franka::NetworkException const& e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
