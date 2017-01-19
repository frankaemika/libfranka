#include "franka/robot.h"

namespace franka {

Robot::Ptr Robot::connect(const std::string&) {
  return std::shared_ptr<Robot>(new Robot);
}

bool Robot::waitForRobotState() {
  return true;
}

RobotState Robot::getRobotState() const {
  return 1;
}

}  // namespace franka
