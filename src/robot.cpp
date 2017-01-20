#include "franka/robot.h"

using namespace franka;

namespace franka {

class Robot::Impl {
 public:
  Impl(const std::string &frankaAddress);

  bool waitForRobotState();
  const RobotState &getRobotState() const;
 private:
  //
};

}  // namespace franka

Robot::Robot(const std::string &frankaAddress)
    : impl_(new Robot::Impl(frankaAddress)) {
}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() = default;

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState &Robot::getRobotState() const {
  return impl_->getRobotState();
}

/* Implementation */

Robot::Impl::Impl(const std::string &frankaAddress) {
  // Connect
}

bool Robot::Impl::waitForRobotState() {
  return false;
}

const RobotState &Robot::Impl::getRobotState() const {
  RobotState state;
  return state;
}

