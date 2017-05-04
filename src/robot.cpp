#include <franka/robot.h>

#include "motion_generator_loop.h"
#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address, RealtimeConfig realtime_config)
    : impl_(new Robot::Impl(franka_address,
                            research_interface::kCommandPort,
                            Robot::Impl::kDefaultTimeout,
                            realtime_config)) {}

// Has to be declared here, as the Impl type is incomplete in the header
Robot::~Robot() noexcept = default;

Robot::Impl& Robot::impl() noexcept {
  return *impl_;
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

void Robot::control(
    std::function<Torques(const RobotState&)> control_callback) {
  ControlLoop loop(*impl_, control_callback);
  loop();
}

void Robot::control(
    std::function<JointValues(const RobotState&)> motion_generator_callback,
    std::function<Torques(const RobotState&)> control_callback) {
  MotionGeneratorLoop<JointValues> loop(*impl_, control_callback,
                                        motion_generator_callback);
  loop();
}

void Robot::control(
    std::function<JointVelocities(const RobotState&)> motion_generator_callback,
    std::function<Torques(const RobotState&)> control_callback) {
  MotionGeneratorLoop<JointVelocities> loop(*impl_, control_callback,
                                            motion_generator_callback);
  loop();
}

void Robot::control(
    std::function<CartesianPose(const RobotState&)> motion_generator_callback,
    std::function<Torques(const RobotState&)> control_callback) {
  MotionGeneratorLoop<CartesianPose> loop(*impl_, control_callback,
                                          motion_generator_callback);
  loop();
}

void Robot::control(
    std::function<CartesianVelocities(const RobotState&)>
        motion_generator_callback,
    std::function<Torques(const RobotState&)> control_callback) {
  MotionGeneratorLoop<CartesianVelocities> loop(*impl_, control_callback,
                                                motion_generator_callback);
  loop();
}

void Robot::read(std::function<bool(const RobotState&)> read_callback) {
  while (impl_->update()) {
    if (!read_callback(impl_->robotState())) {
      break;
    }
  }
}

RobotState Robot::readOnce() {
  if (!impl_->update()) {
    throw NetworkException("libfranka: Disconnected.");
  }
  return impl_->robotState();
}

}  // namespace franka
