#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_joint_pose_motion <robot-hostname>"
              << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop
    // Set a dynamic load:
    double load_mass = 0.1;
    std::array<double, 3> F_x_Cload = {// NOLINT (readability-named-parameter)
                                       0.0, 0.0, 0.0};
    std::array<double, 9> load_inertia = {0.01, 0.0, 0.0, 0.0, 0.01,
                                          0.0,  0.0, 0.0, 0.01};
    robot.setLoad(load_mass, F_x_Cload, load_inertia);

    auto initial_pose = robot.readOnce().q;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&) -> franka::JointValues {
      double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * time));

      time += 0.001;
      if (time > 10.0) {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        return franka::Stop;
      }

      return {{initial_pose[0], initial_pose[1], initial_pose[2],
               initial_pose[3] + delta_angle, initial_pose[4] + delta_angle,
               initial_pose[5], initial_pose[6] + delta_angle}};
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
