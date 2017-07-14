#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_consecutive_motions <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop
    // Set collision behavior:
    robot.setCollisionBehavior(
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}});

    for (int i = 0; i < 5; i++) {
      std::cout << "Executing motion." << std::endl;
      try {
        auto initial_pose = robot.readOnce().q_d;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&) -> franka::JointPositions {
          double delta_angle = M_PI / 32 * (1 - std::cos(M_PI / 1.0 * time));

          time += 0.001;
          if (time > 2.0) {
            std::cout << std::endl << "Finished motion." << std::endl;
            return franka::Stop;
          }

          return {{initial_pose[0], initial_pose[1], initial_pose[2], initial_pose[3],
                   initial_pose[4], initial_pose[5], initial_pose[6] + delta_angle}};
        });
      } catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot.automaticErrorRecovery();
      }
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
