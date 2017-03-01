#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_motion <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    robot.update();
    std::cout << "Starting Cartesian pose motion generator" << std::endl;
    franka::CartesianPoseMotionGenerator motion_generator =
        robot.startCartesianPoseMotionGenerator();
    std::cout << "succeeded to start motion generator! " << std::endl;

    double radius(0.3);
    double time(0.0);
    std::array<double, 3> initial_position{robot.robotState().O_T_EE_start[12],
                                           robot.robotState().O_T_EE_start[13],
                                           robot.robotState().O_T_EE_start[14]};
    while (robot.update()) {
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = radius * std::sin(angle);
      double delta_z = radius * (std::cos(angle) - 1);
      try {
        motion_generator.setDesiredPose(
            {{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
              initial_position[0] + delta_x, initial_position[1],
              initial_position[2] + delta_z, 1.0}});
      } catch (franka::MotionGeneratorException const& e) {
        std::cout << e.what() << std::endl;
      }
      time += 0.001;
      if (time > 10.0) {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        break;
      }
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
