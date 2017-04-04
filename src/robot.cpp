#include <franka/robot.h>

#include <typeinfo>

#include "robot_impl.h"
#include "motion_generator_loop.h"

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

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

void Robot::control(std::function<Torques(const RobotState &)> update_function) {
  ControlLoop loop(*impl_, update_function);
  loop();
}

void Robot::control(std::function<JointValues(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {
  MotionGeneratorLoop<JointValues> loop(*impl_, control_update, motion_generator_update);
  loop();
}

void Robot::control(std::function<JointVelocities(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {
  MotionGeneratorLoop<JointVelocities> loop(*impl_, control_update, motion_generator_update);
  loop();

}

void Robot::control(std::function<CartesianPose(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {
  MotionGeneratorLoop<CartesianPose> loop(*impl_, control_update, motion_generator_update);
  loop();

}

void Robot::control(std::function<CartesianVelocities(const RobotState &)> motion_generator_update,
                    std::function<Torques(const RobotState &)> control_update) {
  MotionGeneratorLoop<CartesianVelocities> loop(*impl_, control_update, motion_generator_update);
  loop();
}

void Robot::read(std::function<void(const RobotState &)> callback) {
  while (impl_->update()) {
    callback(impl_->robotState());
  }
}

RobotState Robot::readOnce() {
  while (!impl_->update()) {}
  return impl_->robotState();
}

}  // namespace franka
