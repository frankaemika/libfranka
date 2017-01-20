#include "franka/robot.h"

namespace franka {

class Robot::Impl {
 public:
  explicit Impl(const std::string& frankaAddress);

  bool waitForRobotState();
  const RobotState& getRobotState() const;

 private:
  RobotState robot_state_;
};

Robot::Robot(const std::string& frankaAddress)
    : impl_(new Robot::Impl(frankaAddress)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() = default;

bool Robot::waitForRobotState() {
  return impl_->waitForRobotState();
}

const RobotState& Robot::getRobotState() const {
  return impl_->getRobotState();
}

/* Implementation */

Robot::Impl::Impl(const std::string& frankaAddress) : robot_state_{} {
  // Connect
}

bool Robot::Impl::waitForRobotState() {
  return false;
}

const RobotState& Robot::Impl::getRobotState() const {
  return robot_state_;
}

}  // namespace franka
