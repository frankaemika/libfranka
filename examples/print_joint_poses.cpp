#include <iostream>
#include <iterator>

#include <franka/model.h>

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1,
            std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./print_joint_poses <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    franka::RobotState state = robot.readOnce();

    franka::Model model(robot);
    for (franka::Joint joint = franka::Joint::kJ0;
         joint <= franka::Joint::kEndEffector;
         joint = franka::Joint(joint + 1)) {
      std::cout << model.jointPose(joint, state) << std::endl;
    }
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
