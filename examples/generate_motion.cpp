#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_motion <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    std::cout << "Starting Motion Generator" << std::endl;
    franka::CartesianPoseMotionGenerator motion_generator =
        robot.startCartesianPoseMotionGenerator();
    std::cout << "succeeded to start motion generator! " << std::endl;

    while (robot.update()) {
      motion_generator.setDesiredPose({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
