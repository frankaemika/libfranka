#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_cartesian_velocity_motion <robot-hostname>"
              << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    std::cout << "Starting Cartesian velocity motion generator" << std::endl;
    franka::CartesianVelocityMotionGenerator motion_generator =
        robot.startCartesianVelocityMotionGenerator();

    double time(0.0);
    double time_max(4.0);
    double v_max(0.1);
    double angle(M_PI / 4);

    while (robot.update()) {
      double cycle =
          std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double v =
          cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
      double v_x = std::cos(angle) * v;
      double v_z = -std::sin(angle) * v;

      motion_generator.setDesiredVelocity({{v_x, 0.0, v_z, 0.0, 0.0, 0.0}});

      time += 0.001;
      if (time > 2 * time_max) {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        break;
      }
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
