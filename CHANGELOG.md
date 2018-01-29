# CHANGELOG

## 0.2.0 - unreleased

Requires Panda system version >= 1.1.0

### Breaking changes

  * New build-time dependency on Eigen3
  * Replaced `p_min` and `p_max` of `franka::VirtualWallCuboid` with `object_world_size`

### New features

  * Added `joint_p2p_insufficient_torque_for_planning`, `tau_j_range_violation`, and
    `instability_detected` errors to `franka::RobotState`
  * Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal`, `I_total`,
    `theta` and `dtheta` to `franka::RobotState`
  * Added logs to `franka::ControlException`
  * Added saturation to joint velocity, joint position, and joint impedance examples
  * Added support for commanding elbow positions for Cartesian motions
  * Fail earlier (by throwing exceptions) if any of the commanded values are NaN or infinity
  * Added support for stiffness frame to `franka::Model`
  * Added overloads in `franka::Model`
  * Added `setFilter` command to `franka::Robot`

### Bugfixes

  * Changed examples to read initial states inside control loops (after controller switching)
  * Always throw `franka::ControlException`s for control-related command responses
  * Prepared for the removal of the socket-init in the default constructor in POCO releases >= 1.8.0
  * Docu: Use SVG instead of MathML for math rendering in API documentation to support Chrome

### Other changes

  * Changes in network protocol for new panda system version
  * Removed unnecessary public dependencies for `libfranka`
  * More descriptive exception messages
  * Examples:
    * CI: Run linter on examples
    * Examples first move to an initial joint position
    * Lowered grasping force in `grasp_object` example
  * Changed thread priority to the maximum allowed value

## 0.1.0 - 2017-09-15

  * Initial release

