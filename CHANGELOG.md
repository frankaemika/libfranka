# CHANGELOG

## 0.8.1 - UNRELEASED

Requires Panda system version >= 4.0.0

### Changed

 * Minor documentation fixes.
 * Bug fixes in `rate_limitng_tests.cpp`.

## 0.8.0 - 2020-04-29

Requires Panda system version >= 4.0.0

 * **BREAKING** Change behavior of `franka::Robot::setEE`. Previously, this method would set the
   flange-to-end-effector transformation `F_T_EE`. This has been split up into two transformations:
   `F_T_NE`, only settable in Desk, and `NE_T_EE`, which can be set in `libfranka` with `setEE` and
   defaults to the identity transformation.
 * Add `F_T_NE` and `NE_T_EE` to `franka::RobotState`.
 * Add support for the cobot pump with `franka::VacuumGripper`.
 * CPack: Add conflict with `ros-melodic-libfranka`.
 * Fix missing include for Windows (#55).

## 0.7.1 - 2019-07-08

Requires Panda system version >= 3.0.0

### Changed

 * Fix compilation error on ARM.

## 0.7.0 - 2019-07-05

Requires Panda system version >= 3.0.0

### Added

 * Add support for using `franka::Model` on Linux ARM and ARM64
   (requires Panda system version >= 3.1.0).
 * Add Cartesian low-pass filter in `lowpass_filter.h`.
 * Add `control_tools.h` with helper functions for writing control loops.

### Changed

 * **BREAKING** Rename `franka::CartesianPose::hasValidElbow()`
   to `franka::CartesianPose::hasElbow()`.
 * **BREAKING** Throw `std::invalid_argument` in `franka::Robot::control` if
   NaN or infinity values are returned from a control callback.
 * **BREAKING** Throw `std::invalid_argument` in low-pass filter and rate limiting
   functions if invalid values are given as function parameters.
 * **BREAKING** Do not throw exceptions in constructors of control types anymore.
 * Take arguments by value in `franka::MotionFinished`.
 * Bug fixes in `communication_test.cpp`.
 * Export requirements for C++ features as CMake target compile features.

## 0.6.0 - 2019-02-06

Requires Panda system version >= 3.0.0

 * Added support for Ubuntu 18.04 (Bionic).
 * **EXPERIMENTAL** Added support for Windows.
 * Added support for using `franka::Model` on Linux and Windows x86 and x64.
 * Bugfix for aborting gripper commands with `franka::Gripper::stop()`.
 * Improved documentation for `franka::Robot::setCollisionBehavior`.

## 0.5.0 - 2018-08-08

Requires Panda system version >= 1.3.0

### Motion and control interfaces

 * **BREAKING** Added low-pass filter for all realtime interfaces with default cutoff frequency
   of 100 Hz
 * **DEPRECATED** `franka::Robot::setFilters` in favor of `franka::lowpassFilter`
 * Fixed description of log record entries

### Library

  * Added `lowpass_filter.h` to public interface

### Examples

 * Added `communication_test.cpp` to evaluate network performance.

## 0.4.0 - 2018-06-21

Requires Panda system version >= 1.3.0

### Motion and control interfaces

  * **BREAKING** Added rate limiting of commanded values as an option in the control loop and as
    a helper function. Rate limiting is activated by default, this could change the behavior of
    existing programs. Only works when filters (`franka::Robot::setFilters`) are deactivated.
  * Added `joint_move_in_wrong_direction` error to `franka::RobotState`
  * Added first and second derivatives of last commanded signals to `franka::RobotState`

### Library

  * Added `rate_limiting.h` to public interface
  * Removed unnecessary CMake script from installation

## 0.3.0 - 2018-02-22

Requires Panda system version >= 1.2.0

### Motion and control interfaces

  * Added optional `epsilon` parameters to `franka::Gripper::grasp`

### Examples

  * Set default collision behavior, impedances, and filter frequencies
  * Removed command line parameters to simplify usage
  * Fixed locking bug in `joint_impedance_control`

### Library

  * **BREAKING** Changed signatures and added overloads in `franka::Model`
  * Added additional variables to installed CMake config
  * Updated `SOVERSION` to include minor version number
  * Added conflict with `ros-kinetic-libfranka` to Debian packaging
  * Minor fixes and improvements for API documentation
  * Fixed build errors on Clang 5.0
  * Fixed test errors for Poco >= 1.8.0

## 0.2.0 - 2018-01-29

Requires Panda system version >= 1.1.0

### Motion and control interfaces

  * Improved external torque tracking behavior.
  * Fixed discontinuities in commanding orientation changes via the cartesian
    pose interface.
  * Added `joint_p2p_insufficient_torque_for_planning`, `tau_j_range_violation`, and
    `instability_detected` errors to `franka::RobotState`
  * Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal`, `I_total`,
    `theta` and `dtheta` to `franka::RobotState`
  * Added `setFilter` command to `franka::Robot`
  * Added support for commanding elbow positions for Cartesian motions.
  * Added stiffness frame `K` to `franka::Model`
  * **BREAKING** Replaced `p_min` and `p_max` of `franka::VirtualWallCuboid` with `object_world_size`

### Error handling

  * **WARNING** Not all robot errors can be recovered using the guiding button
    on the robot anymore. To manually recover from such errors, please use the
    the Franka DESK web interface.
  * Added logs to `ControlException` and improved exception messages.
  * Fail earlier (by throwing exception) if any of the commanded values are
    `NaN` or `infinity`.

### Examples

  * Added saturation to joint velocity, joint position, and joint impedance
    examples.
  * Changed examples to read initial states inside control loops (after
    controller switching).
  * Examples first move to an initial joint position.
  * Added new cartesian impedance and force control examples.
  * Lowered grasping force in `grasp_object` example

### Library

  * **BREAKING** New build-time dependency on Eigen3.
  * Changed thread priority to the maximum allowed value.
  * Prepare for the removal of the socket-init in the default constructor in
    POCO releases >= 1.8.0.
  * Removed unnecessary public dependencies for libfranka.
  * CI: Run linter on examples
  * Docu: Use SVG instead of MathML for math rendering in API documentation to support Chrome.

## 0.1.0 - 2017-09-15

  * Initial release

