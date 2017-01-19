#include "franka/robot.h"

namespace franka {

class Robot::Impl {
 public:
  Impl(const std::string &frankaAddress);

  bool waitForRobotState();
  const RobotState &getRobotState() const;
 private:
  //
};

Robot::Robot(const std::string &frankaAddress)
    : impl_(new Robot::Impl(frankaAddress)) {
}

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState &Robot::getRobotState() const {
  return impl_->getRobotState();
}

Robot::Impl::Impl(const std::string &frankaAddress) {
  // Connect
}

bool Robot::Impl::waitForRobotState() {
  return true;
}

const RobotState &Robot::Impl::getRobotState() const {
  RobotState state;
  return state;
}

}  // namespace franka
