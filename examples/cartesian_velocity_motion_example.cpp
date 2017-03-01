#include <unistd.h>
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
    std::cout << "Starting Cartesian velocity motion generator" << std::endl;
    franka::CartesianVelocityMotionGenerator motion_generator =
        robot.startCartesianVelocityMotionGenerator();
    std::cout << "succeeded to start motion generator! " << std::endl;
    double time(0.0);
    double T(4.0);
    double v_max(0.1);
    double angle(M_PI / 4);

    while (robot.update()) {
      int cycle = int(pow(-1.0, (time - fmod(time, T)) / T));
      double v =
          double(cycle) * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / T * time));
      double v_x = std::cos(angle) * v;
      double v_z = -std::sin(angle) * v;
      try {
        motion_generator.setDesiredVelocity({{v_x, 0.0, v_z, 0.0, 0.0, 0.0}});
      } catch (franka::MotionGeneratorException const& e) {
        std::cout << e.what() << std::endl;
      }
      time += 0.001;
      if (time > 2 * T) {
        std::cout << std::endl
                  << "finished motion, shutting down example" << std::endl;
        break;
      }
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
