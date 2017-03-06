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
    robot.update();
    std::cout << "Starting joint pose motion generator" << std::endl;
    franka::JointPoseMotionGenerator motion_generator =
        robot.startJointPoseMotionGenerator();

    double time(0.0);
    std::array<double, 7> initial_pose;
    std::copy(robot.robotState().q.cbegin(), robot.robotState().q.cend(),
              initial_pose.begin());

    while (robot.update()) {
      double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * time));

      motion_generator.setDesiredPose(
          {{initial_pose[0], initial_pose[1], initial_pose[2],
            initial_pose[3] + delta_angle, initial_pose[4] + delta_angle,
            initial_pose[5], initial_pose[6] + delta_angle}});

      time += 0.001;
      if (time > 10.0) {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        break;
      }
    }
  } catch (const franka::NetworkException& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
