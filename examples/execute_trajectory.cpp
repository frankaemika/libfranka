#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <franka/robot.h>

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
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0]
              << " <robot-hostname> <trajectory-csv> <output>" << std::endl;
    return -1;
  }

  std::cout << "Loading csv trajectory" << std::endl;
  std::fstream csv_file_stream;
  csv_file_stream.open(argv[2], std::fstream::in);

  std::vector<std::array<double, 7>> samples;
  while (csv_file_stream) {
    std::array<double, 7> q;
    char delimiter;
    for (int i = 0; i < 7; i++) {
      csv_file_stream >> q[i] >> delimiter;
    }
    samples.push_back(q);
  }
  std::cout << "Read " << samples.size() << " samples" << std::endl;

  std::vector<franka::RobotState> states;
  try {
    franka::Robot robot(argv[1]);
    robot.update();
    std::cout << "Starting joint pose motion generator" << std::endl;
    franka::JointPoseMotionGenerator motion_generator =
        robot.startJointPoseMotionGenerator();

    for (std::array<double, 7>& q : samples) {
      if (!robot.update()) {
        std::cout << "Connection lost" << std::endl;
        break;
      }
      motion_generator.setDesiredPose(q);
      states.push_back(robot.robotState());
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (franka::MotionGeneratorException const& e) {
    std::cout << e.what() << std::endl;
  }

  std::cout << "Logging results to file: " << argv[3] << std::endl;
  std::fstream output_stream;
  output_stream.open(argv[3], std::fstream::out);
  for (unsigned int s = 0; s < states.size(); s++) {
    output_stream << "Sample: #" << s << std::endl;
    output_stream << "q_d: \t" << samples[s] << std::endl;
    output_stream << "Robot state:" << std::endl;
    output_stream << "q: \t" << states[s].q << std::endl;
    output_stream << "q_d: \t" << states[s].q_d << std::endl;
    output_stream << "dq: \t" << states[s].dq << std::endl;
    output_stream << "______" << std::endl;
  }

  return 0;
}
