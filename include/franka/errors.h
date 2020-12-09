// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <ostream>

/**
 * @file errors.h
 * Contains the franka::Errors type.
 */

namespace franka {

/**
 * Enumerates errors that can occur while controlling a franka::Robot.
 */
struct Errors {
 private:
  static constexpr int kMaxNumErrors = 37;
  std::array<bool, kMaxNumErrors> errors_{};

 public:
  const std::array<bool, kMaxNumErrors>& errors() const { return errors_; }

  /**
   * Creates an empty Errors instance.
   */
  Errors() = default;

  /**
   * Copy constructs a new Errors instance.
   *
   * @param[in] other Other Errors instance.
   */
  Errors(const Errors& other) = default;

  /**
   * Assigns this Errors instance from another Errors value.
   *
   * @param[in] other Other Errors instance.
   *
   * @return Errors instance.
   */
  Errors& operator=(const Errors& other) = default;

  /**
   * Creates a new Errors instance from the given array.
   *
   * @param errors Array of error flags.
   */
  Errors(const std::array<bool, 37>& errors);

  /**
   * Check if any error flag is set to true.
   *
   * @return True if any errors are set.
   */
  explicit operator bool() const noexcept;

  /**
   * Creates a string with names of active errors:
   * "[active_error_name2, active_error_name_2, ... active_error_name_n]"
   * If no errors are active, the string contains empty brackets: "[]"
   *
   * @return string with names of active errors
   */
  explicit operator std::string() const;

  /**
   * True if the robot moved past the joint limits.
   */
  const bool& joint_position_limits_violation() const;
  /**
   * True if the robot moved past any of the virtual walls.
   */
  const bool& cartesian_position_limits_violation() const;
  /**
   * True if the robot would have collided with itself.
   */
  const bool& self_collision_avoidance_violation() const;
  /**
   * True if the robot exceeded joint velocity limits.
   */
  const bool& joint_velocity_violation() const;
  /**
   * True if the robot exceeded Cartesian velocity limits.
   */
  const bool& cartesian_velocity_violation() const;
  /**
   * True if the robot exceeded safety threshold during force control.
   */
  const bool& force_control_safety_violation() const;
  /**
   * True if a collision was detected, i.e.\ the robot exceeded a torque threshold in a joint
   * motion.
   */
  const bool& joint_reflex() const;
  /**
   * True if a collision was detected, i.e.\ the robot exceeded a torque threshold in a Cartesian
   * motion.
   */
  const bool& cartesian_reflex() const;
  /**
   * True if internal motion generator did not reach the goal pose.
   */
  const bool& max_goal_pose_deviation_violation() const;
  /**
   * True if internal motion generator deviated from the path.
   */
  const bool& max_path_pose_deviation_violation() const;
  /**
   * True if Cartesian velocity profile for internal motions was exceeded.
   */
  const bool& cartesian_velocity_profile_safety_violation() const;
  /**
   * True if an external joint position motion generator was started with a pose too far from the
   * current pose.
   */
  const bool& joint_position_motion_generator_start_pose_invalid() const;
  /**
   * True if an external joint motion generator would move into a joint limit.
   */
  const bool& joint_motion_generator_position_limits_violation() const;
  /**
   * True if an external joint motion generator exceeded velocity limits.
   */
  const bool& joint_motion_generator_velocity_limits_violation() const;
  /**
   * True if commanded velocity in joint motion generators is discontinuous (target values are too
   * far apart).
   */
  const bool& joint_motion_generator_velocity_discontinuity() const;
  /**
   * True if commanded acceleration in joint motion generators is discontinuous (target values are
   * too far apart).
   */
  const bool& joint_motion_generator_acceleration_discontinuity() const;
  /**
   * True if an external Cartesian position motion generator was started with a pose too far from
   * the current pose.
   */
  const bool& cartesian_position_motion_generator_start_pose_invalid() const;
  /**
   * True if an external Cartesian motion generator would move into an elbow limit.
   */
  const bool& cartesian_motion_generator_elbow_limit_violation() const;
  /**
   * True if an external Cartesian motion generator would move with too high velocity.
   */
  const bool& cartesian_motion_generator_velocity_limits_violation() const;
  /**
   * True if commanded velocity in Cartesian motion generators is discontinuous (target values are
   * too far apart).
   */
  const bool& cartesian_motion_generator_velocity_discontinuity() const;
  /**
   * True if commanded acceleration in Cartesian motion generators is discontinuous (target values
   * are too far apart).
   */
  const bool& cartesian_motion_generator_acceleration_discontinuity() const;
  /**
   * True if commanded elbow values in Cartesian motion generators are inconsistent.
   */
  const bool& cartesian_motion_generator_elbow_sign_inconsistent() const;
  /**
   * True if the first elbow value in Cartesian motion generators is too far from initial one.
   */
  const bool& cartesian_motion_generator_start_elbow_invalid() const;
  /**
   * True if the joint position limits would be exceeded after IK calculation.
   */
  const bool& cartesian_motion_generator_joint_position_limits_violation() const;
  /**
   * True if the joint velocity limits would be exceeded after IK calculation.
   */
  const bool& cartesian_motion_generator_joint_velocity_limits_violation() const;
  /**
   * True if the joint velocity in Cartesian motion generators is discontinuous after IK
   * calculation.
   */
  const bool& cartesian_motion_generator_joint_velocity_discontinuity() const;
  /**
   * True if the joint acceleration in Cartesian motion generators is discontinuous after IK
   * calculation.
   */
  const bool& cartesian_motion_generator_joint_acceleration_discontinuity() const;
  /**
   * True if the Cartesian pose is not a valid transformation matrix.
   */
  const bool& cartesian_position_motion_generator_invalid_frame() const;
  /**
   * True if desired force exceeds the safety thresholds.
   */
  const bool& force_controller_desired_force_tolerance_violation() const;
  /**
   * True if the torque set by the external controller is discontinuous.
   */
  const bool& controller_torque_discontinuity() const;
  /**
   * True if the start elbow sign was inconsistent.
   *
   * Applies only to motions started from Desk.
   */
  const bool& start_elbow_sign_inconsistent() const;
  /**
   * True if minimum network communication quality could not be held during a motion.
   */
  const bool& communication_constraints_violation() const;
  /**
   * True if commanded values would result in exceeding the power limit.
   */
  const bool& power_limit_violation() const;
  /**
   * True if the robot is overloaded for the required motion.
   *
   * Applies only to motions started from Desk.
   */
  const bool& joint_p2p_insufficient_torque_for_planning() const;
  /**
   * True if the measured torque signal is out of the safe range.
   */
  const bool& tau_j_range_violation() const;
  /**
   * True if an instability is detected.
   */
  const bool& instability_detected() const;
  /**
   * True if the robot is in joint position limits violation error and the user guides the robot
   * further towards the limit.
   */
  const bool& joint_move_in_wrong_direction() const;
};

/**
 * Streams the errors as JSON array.
 *
 * @param[in] ostream Ostream instance
 * @param[in] errors Errors struct instance to stream
 *
 * @return Ostream instance
 */
std::ostream& operator<<(std::ostream& ostream, const Errors& errors);

}  // namespace franka
