#include <franka/robot.h>

#include <typeinfo>

#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address)
    : impl_(new Robot::Impl(franka_address)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() noexcept = default;

Robot::Impl& Robot::impl() noexcept {
  return *impl_;
}

bool Robot::update() {
  return impl_->update();
}

const RobotState& Robot::robotState() const noexcept {
  return impl_->robotState();
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

CartesianPoseMotionGenerator Robot::startCartesianPoseMotionGenerator() {
  return CartesianPoseMotionGenerator(*this);
}

CartesianVelocityMotionGenerator
Robot::startCartesianVelocityMotionGenerator() {
  return CartesianVelocityMotionGenerator(*this);
}

JointPoseMotionGenerator Robot::startJointPoseMotionGenerator() {
  return JointPoseMotionGenerator(*this);
}

JointVelocityMotionGenerator Robot::startJointVelocityMotionGenerator() {
  return JointVelocityMotionGenerator(*this);
}

void Robot::control(std::function<Torques(const RobotState &)> update_function) {
  if(!update_function) {
    throw std::invalid_argument("libfranka: empty update function");
  }

  impl_->startController();

  while(impl_->update()) {
    Torques t = update_function(impl_->robotState());
    if(typeid(t) == typeid(Stop)) {
      break;
    }

    auto& command = impl_->controllerCommand();
    //command.tau_J_d = t;
  }

  impl_->stopController();
}

void Robot::control(std::function<JointValues(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {

}
void Robot::control(std::function<JointVelocities(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {

}
void Robot::control(std::function<CartesianPose(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {

}
void Robot::control(std::function<CartesianVelocities(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {

}
void Robot::read(std::function<void(const RobotState &)> update_function) {

}

}  // namespace franka
