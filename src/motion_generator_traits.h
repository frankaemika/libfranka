#pragma once

#include <franka/control_types.h>
#include <research_interface/service_types.h>

namespace franka {

template <typename T>
struct MotionGeneratorTraits {};

template <>
struct MotionGeneratorTraits<JointValues> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::StartMotionGenerator::MotionGeneratorMode::kJointPosition;
};

template <>
struct MotionGeneratorTraits<JointVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::StartMotionGenerator::MotionGeneratorMode::kJointVelocity;
};

template <>
struct MotionGeneratorTraits<CartesianPose> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::StartMotionGenerator::MotionGeneratorMode::kCartesianPosition;
};

template <>
struct MotionGeneratorTraits<CartesianVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::StartMotionGenerator::MotionGeneratorMode::kCartesianVelocity;
};

}  // namespace franka
