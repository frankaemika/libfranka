#pragma once

#include <array>
#include <ostream>

namespace franka {

struct Errors {
  Errors() = default;
  Errors(std::array<bool, 24> errors);

  explicit operator bool() const noexcept;

  bool joint_position_limits_violation;
  bool cartesian_position_limits_violation;
  bool self_collision_avoidance_violation;
  bool joint_velocity_violation;
  bool cartesian_velocity_violation;
  bool force_control_safety_violation;
  bool joint_reflex;
  bool cartesian_reflex;
  bool max_goal_pose_deviation_violation;
  bool max_path_pose_deviation_violation;
  bool cartesian_velocity_profile_safety_violation;
  bool joint_position_motion_generator_start_pose_invalid;
  bool joint_motion_generator_position_limits_violation;
  bool joint_motion_generator_velocity_limits_violation;
  bool joint_motion_generator_velocity_discontinuity;
  bool joint_motion_generator_acceleration_discontinuity;
  bool cartesian_position_motion_generator_start_pose_invalid;
  bool cartesian_motion_generator_elbow_limit_violation;
  bool cartesian_motion_generator_velocity_limits_violation;
  bool cartesian_motion_generator_velocity_discontinuity;
  bool cartesian_motion_generator_acceleration_discontinuity;
  bool cartesian_motion_generator_elbow_sign_inconsistent;
  bool cartesian_motion_generator_start_elbow_invalid;
  bool force_controller_desired_force_tolerance_violation;
};

std::string activeErrorsString(Errors& errors);

std::ostream& operator<<(std::ostream& ostream, const Errors& errors);
bool operator==(const Errors& lhs, const Errors& rhs);

}  // namespace franka