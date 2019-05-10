// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/robot.h>

#include <utility>

#include "control_loop.h"
#include "network.h"
#include "robot_impl.h"

namespace franka {

Robot::Robot(const std::string& franka_address, RealtimeConfig realtime_config, size_t log_size)
    : impl_{new Robot::Impl(
          std::make_unique<Network>(franka_address, research_interface::robot::kCommandPort),
          log_size,
          realtime_config)} {}

// Has to be declared here, as the Impl type is incomplete in the header.
Robot::~Robot() noexcept = default;

Robot::Robot(Robot&& other) noexcept {
  std::lock_guard<std::mutex> _(other.control_mutex_);
  impl_ = std::move(other.impl_);
}

Robot& Robot::operator=(Robot&& other) noexcept {
  if (&other != this) {
    std::unique_lock<std::mutex> this_lock(control_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> other_lock(other.control_mutex_, std::defer_lock);
    std::lock(this_lock, other_lock);
    impl_ = std::move(other.impl_);
  }
  return *this;
}

Robot::ServerVersion Robot::serverVersion() const noexcept {
  return impl_->serverVersion();
}

void Robot::control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<JointVelocities> loop(*impl_, std::move(control_callback),
                                    [](const RobotState&, Duration) -> JointVelocities {
                                      return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                                    },
                                    limit_rate, cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<JointPositions> loop(*impl_, std::move(control_callback),
                                   std::move(motion_generator_callback), limit_rate,
                                   cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<JointVelocities> loop(*impl_, std::move(control_callback),
                                    std::move(motion_generator_callback), limit_rate,
                                    cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<Torques(const RobotState&, franka::Duration)> control_callback,
    std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<CartesianPose> loop(*impl_, std::move(control_callback),
                                  std::move(motion_generator_callback), limit_rate,
                                  cutoff_frequency);
  loop();
}

void Robot::control(std::function<Torques(const RobotState&, franka::Duration)> control_callback,
                    std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                        motion_generator_callback,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<CartesianVelocities> loop(*impl_, std::move(control_callback),
                                        std::move(motion_generator_callback), limit_rate,
                                        cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<JointPositions(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<JointPositions> loop(*impl_, controller_mode, std::move(motion_generator_callback),
                                   limit_rate, cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<JointVelocities(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<JointVelocities> loop(*impl_, controller_mode, std::move(motion_generator_callback),
                                    limit_rate, cutoff_frequency);
  loop();
}

void Robot::control(
    std::function<CartesianPose(const RobotState&, franka::Duration)> motion_generator_callback,
    ControllerMode controller_mode,
    bool limit_rate,
    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<CartesianPose> loop(*impl_, controller_mode, std::move(motion_generator_callback),
                                  limit_rate, cutoff_frequency);
  loop();
}

void Robot::control(std::function<CartesianVelocities(const RobotState&, franka::Duration)>
                        motion_generator_callback,
                    ControllerMode controller_mode,
                    bool limit_rate,
                    double cutoff_frequency) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  ControlLoop<CartesianVelocities> loop(
      *impl_, controller_mode, std::move(motion_generator_callback), limit_rate, cutoff_frequency);
  loop();
}

// NOLINTNEXTLINE(performance-unnecessary-value-param)
void Robot::read(std::function<bool(const RobotState&)> read_callback) {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  while (true) {
    RobotState robot_state = impl_->update(nullptr, nullptr);
    if (!read_callback(robot_state)) {
      break;
    }
  }
}

RobotState Robot::readOnce() {
  std::unique_lock<std::mutex> l(control_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka robot: Cannot perform this operation while another control or read operation "
        "is running.");
  }

  return impl_->readOnce();
}

VirtualWallCuboid Robot::getVirtualWall(int32_t id) {
  VirtualWallCuboid virtual_wall;
  impl_->executeCommand<research_interface::robot::GetCartesianLimit>(id, &virtual_wall);
  return virtual_wall;
}

void Robot::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                                 const std::array<double, 7>& upper_torque_thresholds_acceleration,
                                 const std::array<double, 7>& lower_torque_thresholds_nominal,
                                 const std::array<double, 7>& upper_torque_thresholds_nominal,
                                 const std::array<double, 6>& lower_force_thresholds_acceleration,
                                 const std::array<double, 6>& upper_force_thresholds_acceleration,
                                 const std::array<double, 6>& lower_force_thresholds_nominal,
                                 const std::array<double, 6>& upper_force_thresholds_nominal) {
  impl_->executeCommand<research_interface::robot::SetCollisionBehavior>(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void Robot::setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                                 const std::array<double, 7>& upper_torque_thresholds,
                                 const std::array<double, 6>& lower_force_thresholds,
                                 const std::array<double, 6>& upper_force_thresholds) {
  impl_->executeCommand<research_interface::robot::SetCollisionBehavior>(
      lower_torque_thresholds, upper_torque_thresholds, lower_torque_thresholds,
      upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds,
      lower_force_thresholds, upper_force_thresholds);
}

void Robot::setJointImpedance(
    const std::array<double, 7>& K_theta) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetJointImpedance>(K_theta);
}

void Robot::setCartesianImpedance(
    const std::array<double, 6>& K_x) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetCartesianImpedance>(K_x);
}

void Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow) {
  impl_->executeCommand<research_interface::robot::SetGuidingMode>(guiding_mode, elbow);
}

void Robot::setK(const std::array<double, 16>& EE_T_K) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetEEToK>(EE_T_K);
}

void Robot::setEE(const std::array<double, 16>& NE_T_EE) {  // NOLINT(readability-identifier-naming)
  impl_->executeCommand<research_interface::robot::SetNEToEE>(NE_T_EE);
}

void Robot::setLoad(
    double load_mass,
    const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
    const std::array<double, 9>& load_inertia) {
  impl_->executeCommand<research_interface::robot::SetLoad>(load_mass, F_x_Cload, load_inertia);
}

void Robot::setFilters(double joint_position_filter_frequency,
                       double joint_velocity_filter_frequency,
                       double cartesian_position_filter_frequency,
                       double cartesian_velocity_filter_frequency,
                       double controller_filter_frequency) {
  impl_->executeCommand<research_interface::robot::SetFilters>(
      joint_position_filter_frequency, joint_velocity_filter_frequency,
      cartesian_position_filter_frequency, cartesian_velocity_filter_frequency,
      controller_filter_frequency);
}

void Robot::automaticErrorRecovery() {
  impl_->executeCommand<research_interface::robot::AutomaticErrorRecovery>();
}

void Robot::stop() {
  impl_->executeCommand<research_interface::robot::StopMove>();
}

Model Robot::loadModel() {
  return impl_->loadModel();
}

}  // namespace franka
