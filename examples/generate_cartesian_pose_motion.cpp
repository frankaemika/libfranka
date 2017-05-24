#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_cartesian_pose_motion <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    auto initial_pose = robot.readOnce().O_T_EE;
    double radius = 0.3;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&) -> franka::CartesianPose {
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = radius * std::sin(angle);
      double delta_z = radius * (std::cos(angle) - 1);

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;

      time += 0.001;
      if (time > 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::Stop;
      }

      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
