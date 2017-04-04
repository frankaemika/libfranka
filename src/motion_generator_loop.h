#pragma once

#include "control_loop.h"
#include "motion_traits.h"

namespace franka {

template <typename T>
class MotionGeneratorLoop : public ControlLoop {
 public:
  using MotionGeneratorCallback = std::function<T(const RobotState&)>;

  MotionGeneratorLoop(Robot::Impl& robot_impl,
                      ControlCallback control_callback,
                      MotionGeneratorCallback motion_callback);

  ~MotionGeneratorLoop() override;

  /**
   * Checks a cartesian pose command (a homogeneous transformation) for
   * validity.
   *
   * @param[in] transform Homogeneous transformation to be checked,
   * passed as column major array
   * @return True if transformation has ortho-normal rotation matrix,
   * the last row is [0 0 0 1] and the array defines a column major matrix
   */
  static bool checkHomogeneousTransformation(
      std::array<double, 16> transform) noexcept;

 protected:
  bool spinOnce() override;

 private:
  void convertMotion(const T& motion,
                     research_interface::MotionGeneratorCommand* command);

  MotionGeneratorCallback motion_callback_;
};

template class MotionGeneratorLoop<JointValues>;
template class MotionGeneratorLoop<JointVelocities>;
template class MotionGeneratorLoop<CartesianPose>;
template class MotionGeneratorLoop<CartesianVelocities>;

}  // namespace franka
