#include <franka/robot.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    size_t count = 0;
    robot.read([&count](const franka::RobotState& robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
      // but should not be done in a control loop.
      std::cout << robot_state << std::endl;
      return count++ < 100;
    });

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
