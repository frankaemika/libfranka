#pragma once

#include <memory>

namespace franka {

class CartesianPoseMotionGenerator {
 public:
  class Impl;

  CartesianPoseMotionGenerator(
      CartesianPoseMotionGenerator&& motion_generator) noexcept;
  explicit CartesianPoseMotionGenerator(Impl&& impl);
  ~CartesianPoseMotionGenerator() noexcept;

  void setDesiredPose(const std::array<double, 16>& desired_pose) noexcept;

  CartesianPoseMotionGenerator() = delete;
  CartesianPoseMotionGenerator(const CartesianPoseMotionGenerator&) = delete;
  CartesianPoseMotionGenerator& operator=(const CartesianPoseMotionGenerator&) =
      delete;
  CartesianPoseMotionGenerator& operator=(
      const CartesianPoseMotionGenerator&&) = delete;

 private:
  std::unique_ptr<Impl> impl_;
};

// class CartesianVelocityMotionGenerator : public MotionGenerator {
// public:
//   CartesianVelocityMotionGenerator() = delete;
//   CartesianVelocityMotionGenerator(const CartesianVelocityMotionGenerator&) =
//   delete;
//   using MotionGenerator::MotionGenerator;
//   void setDesiredVelocity(std::array<double, 6>& desired_velocity);
// };
//
//
// class JointPoseMotionGenerator : public MotionGenerator {
// public:
//   JointPoseMotionGenerator() = delete;
//   JointPoseMotionGenerator(const JointPoseMotionGenerator&) = delete;
//   using MotionGenerator::MotionGenerator;
//   void setDesiredPose(std::array<double, 7>& desired_pose);
//   bool checkPose(std::array<double, 7>& desired_pose);
// };
//
//
// class JointVelocityMotionGenerator : public MotionGenerator {
// public:
//   JointVelocityMotionGenerator() = delete;
//   JointVelocityMotionGenerator(const JointVelocityMotionGenerator&) = delete;
//   using MotionGenerator::MotionGenerator;
//   void setDesiredVelocity(std::array<double, 7>& desired_velocity);
// };

}  // namespace franka
