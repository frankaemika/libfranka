#pragma once

#include <array>
#include <ostream>

/**
 * @file errors.h
 * Contains the Error struct.
 */

namespace franka {

/**
 * Enumerates FRANKA's error flags.
 */
struct Errors {
  Errors() = default;
  /**
   * Sets the error according to values from the array
   * @param errors array of error flags
   */
  Errors(std::array<bool, 25> errors);

  /**
   * Check if any error flag is set to true
   * @return true if any errors are set
   */
  explicit operator bool() const noexcept;

  /**
   * Creates a string with names of active errors:
   * "{active_error_name2, active_error_name_2, ... active_error_name_n}"
   * If no errors are active, the string contains empty braces: "{}"
   * @return string with names of active errors
   */
  explicit operator std::string() const;

  /**
   * True if FRANKA moved past the joint limits.
   */
  bool joint_position_limits_violation{};
  /**
   * True if FRANKA moved past any of the virtual walls.
   */
  bool cartesian_position_limits_violation{};
  /**
   * True if FRANKA would have collided with itself.
   */
  bool self_collision_avoidance_violation{};
  /**
   * True if FRANKA exceeded joint velocity limits.
   */
  bool joint_velocity_violation{};
  /**
   * True if FRANKA exceeded cartesian velocity limits.
   */
  bool cartesian_velocity_violation{};
  /**
   * True if FRANKA exceeded safety threshold during force control.
   */
  bool force_control_safety_violation{};
  /**
   * True if a collision was detected: i.e. FRANKA exceeded torque threshold in a joint motion.
   */
  bool joint_reflex{};
  /**
   * True if a collision was detected: i.e. FRANKA exceeded torque threshold in a Cartesian motion.
   */
  bool cartesian_reflex{};
  /**
   * True if internal motion generator did not reach the goal pose.
   */
  bool max_goal_pose_deviation_violation{};
  /**
   * True if internal motion generator deviated from the path.
   */
  bool max_path_pose_deviation_violation{};
  /**
   * True if cartesian velocity profile for internal motions was exceeded.
   */
  bool cartesian_velocity_profile_safety_violation{};
  /**
   * True if an external joint position motion generator was started with a pose too far from the
   * current pose.
   */
  bool joint_position_motion_generator_start_pose_invalid{};
  /**
   * True if an external joint motion generator would move into a joint limit.
   */
  bool joint_motion_generator_position_limits_violation{};
  /**
   * True if an external joint motion generator exceeded velocity limits.
   */
  bool joint_motion_generator_velocity_limits_violation{};
  /**
   * True if commanded velocity in joint motion generators is discontinuous (target values are too
   * far apart).
   */
  bool joint_motion_generator_velocity_discontinuity{};
  /**
   * True if commanded acceleration in joint motion generators is discontinuous (target values are
   * too far apart).
   */
  bool joint_motion_generator_acceleration_discontinuity{};
  /**
   * True if an external Cartesian position motion generator was started with a pose too far from
   * the current pose.
   */
  bool cartesian_position_motion_generator_start_pose_invalid{};
  /**
   * True if an external Cartesian motion generator would move into an elbow limit.
   */
  bool cartesian_motion_generator_elbow_limit_violation{};
  /**
   * True if an external Cartesian motion generator would move with too high velocity.
   */
  bool cartesian_motion_generator_velocity_limits_violation{};
  /**
   * True if commanded velocity in Cartesian motion generators is discontinuous (target values are
   * too far apart).
   */
  bool cartesian_motion_generator_velocity_discontinuity{};
  /**
   * True if commanded acceleration in Cartesian motion generators is discontinuous (target values
   * are too far apart).
   */
  bool cartesian_motion_generator_acceleration_discontinuity{};
  /**
   * True if commanded elbow values in Cartesian motion generators are inconsistent.
   */
  bool cartesian_motion_generator_elbow_sign_inconsistent{};
  /**
   * True if the first elbow value in Cartesian motion generators is too far from initial one.
   */
  bool cartesian_motion_generator_start_elbow_invalid{};
  /**
   * True if desired force exceeds the safety thresholds.
   */
  bool force_controller_desired_force_tolerance_violation{};
};

/**
 * Streams the Error struct in JSON format.
 */
std::ostream& operator<<(std::ostream& ostream, const Errors& errors);

}  // namespace franka