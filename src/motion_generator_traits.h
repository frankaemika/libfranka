#pragma once

#include <franka/control_types.h>
#include <research_interface/service_types.h>

namespace franka {

template <typename T>
struct MotionGeneratorTraits {};

template <>
struct MotionGeneratorTraits<JointPositions> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::Move::MotionGeneratorMode::kJointPosition;
};

template <>
struct MotionGeneratorTraits<JointVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::Move::MotionGeneratorMode::kJointVelocity;
};

template <>
struct MotionGeneratorTraits<CartesianPose> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::Move::MotionGeneratorMode::kCartesianPosition;
};

template <>
struct MotionGeneratorTraits<CartesianVelocities> {
  static constexpr auto kMotionGeneratorMode =
      research_interface::Move::MotionGeneratorMode::kCartesianVelocity;
};

}  // namespace franka
