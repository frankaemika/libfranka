#include <franka/errors.h>

#include <algorithm>

namespace franka {

namespace {
const std::array<std::string, 24> kNames = {
    {"joint_position_limits_violation",
     "cartesian_position_limits_violation",
     "self_collision_avoidance_violation",
     "joint_velocity_violation",
     "cartesian_velocity_violation",
     "force_control_safety_violation",
     "joint_reflex_flag",
     "cartesian_reflex_flag",
     "max_goal_pose_deviation_violation_flag",
     "max_path_pose_deviation_violation_flag",
     "cartesian_velocity_profile_safety_violation_flag",
     "fci_joint_position_motion_generator_start_pose_invalid_flag",
     "fci_joint_motion_generator_position_limits_violation_flag",
     "fci_joint_motion_generator_velocity_limits_violation_flag",
     "fci_joint_motion_generator_velocity_discontinuity_flag",
     "fci_joint_motion_generator_acceleration_discontinuity_flag",
     "fci_cartesian_position_motion_generator_start_pose_invalid_flag",
     "fci_cartesian_motion_generator_elbow_limit_violation_flag",
     "fci_cartesian_motion_generator_velocity_limits_violation_flag",
     "fci_cartesian_motion_generator_velocity_discontinuity_flag",
     "fci_cartesian_motion_generator_acceleration_discontinuity_flag",
     "fci_cartesian_motion_generator_elbow_sign_inconsistent_flag",
     "fci_cartesian_motion_generator_start_elbow_invalid_flag",
     "force_controller_desired_force_tolerance_violation_flag"}};
}  // anonymous namespace

Errors::Errors(std::array<bool, 24> errors) : errors_(errors) {}  // NOLINT

bool Errors::any() const {
  return std::any_of(errors_.cbegin(), errors_.cend(), [](bool error) { return error; });
}

bool Errors::violated(ErrorType error) const {
  if (error >= kErrorTypeEndMarker) {
    throw std::out_of_range("Error value out of range");
  }
  return errors_[static_cast<int>(error)];
}

std::string Errors::name(ErrorType error) const {
  if (error >= kErrorTypeEndMarker) {
    throw std::out_of_range("Error value out of range");
  }
  return kNames[static_cast<int>(error)];
}

std::string Errors::namesViolated() const {
  std::string names;

  for (ErrorType e = kJointPositionLimitsViolation; e < kErrorTypeEndMarker; e = ErrorType(e + 1)) {
    if (violated(e)) {
      if (!names.empty()) {
        names += ", ";
      }
      names += name(e);
    }
  }
  return names;
}

std::ostream& operator<<(std::ostream& ostream, const Errors& errors) {
  ostream << "{";
  for (ErrorType e = kJointPositionLimitsViolation; e < kErrorTypeEndMarker; e = ErrorType(e + 1)) {
    if (e != kJointPositionLimitsViolation) {
      ostream << ", ";
    }
    ostream << errors.name(e) << ": " << errors.violated(e);
  }
  ostream << "}";
  return ostream;
}

}  // namespace franka
