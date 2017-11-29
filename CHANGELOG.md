# CHANGELOG

## 0.2.0 - unreleased

Requires Panda system version > 1.0.9

### New features

  * Added two new errors to the robot state
  * Added `epsilon` parameters to gripper grasp method
  * Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal` and `I_total`
    to the robot state
  * Added logs to ControlException
  * Added saturation to joint velocity, joint position, and joint impedance examples
  * Added support for commanding elbow positions for Cartesian motions
  * Fail earlier (by throwing exception) if any of the commanded values are NaN or infinity
  * Added support for stiffness frame to model

### Bugfixes

  * Changed examples to read initial states inside control loops (after controller switching)
  * Always throw `ControlException`s for control-related command responses

### Other changes

  * Changes in network protocol for new panda system version
  * Removed unnecessary public dependencies for libfranka

## 0.1.0 - 2017-09-15

  * Initial release

