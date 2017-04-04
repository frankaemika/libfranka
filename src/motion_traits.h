#pragma once

#include <franka/control_types.h>
#include <research_interface/types.h>

namespace franka {

template <typename T>
struct MotionTraits {};

template <>
struct MotionTraits<JointValues> {
  static constexpr research_interface::StartMotionGeneratorRequest::Type
      kMotionGeneratorType =
          research_interface::StartMotionGeneratorRequest::Type::kJointPosition;
};

template <>
struct MotionTraits<JointVelocities> {
  static constexpr research_interface::StartMotionGeneratorRequest::Type
      kMotionGeneratorType =
          research_interface::StartMotionGeneratorRequest::Type::kJointVelocity;
};

template <>
struct MotionTraits<CartesianPose> {
  static constexpr research_interface::StartMotionGeneratorRequest::Type
      kMotionGeneratorType = research_interface::StartMotionGeneratorRequest::
          Type::kCartesianPosition;
};

template <>
struct MotionTraits<CartesianVelocities> {
  static constexpr research_interface::StartMotionGeneratorRequest::Type
      kMotionGeneratorType = research_interface::StartMotionGeneratorRequest::
          Type::kCartesianVelocity;
};

}  // namespace franka
