#pragma once

#include <array>
#include <ostream>

namespace franka {

enum ErrorType {
  kJointPositionLimitsViolation,
  kSelfCollisionAvoidanceViolation,
  kJointVelocityViolation,
  kCartesianVelocityViolation,
  kForceControlSafetyViolation,
  kJointReflexFlag,
  kCartesianReflexFlag,
  kMaxGoalPoseDeviationViolationFlag,
  kMaxPathPoseDeviationViolationFlag,
  kCartesianVelocityProfileSafetyViolationFlag,
  kFciJointPositionMotionGeneratorStartPoseInvalidFlag,
  kFciJointMotionGeneratorPositionLimitsViolationFlag,
  kFciJointMotionGeneratorVelocityLimitsViolationFlag,
  kFciJointMotionGeneratorVelocityDiscontinuityFlag,
  kFciJointMotionGeneratorAccelerationDiscontinuityFlag,
  kFciCartesianPositionMotionGeneratorStartPoseInvalidFlag,
  kFciCartesianMotionGeneratorElbowLimitViolationFlag,
  kFciCartesianMotionGeneratorVelocityLimitsViolationFlag,
  kFciCartesianMotionGeneratorVelocityDiscontinuityFlag,
  kFciCartesianMotionGeneratorAccelerationDiscontinuityFlag,
  kFciCartesianMotionGeneratorElbowSignInconsistentFlag,
  kFciCartesianMotionGeneratorStartElbowInvalidFlag,
  kForceControllerDesiredForceToleranceViolationFlag,
  kErrorTypeEndMarker
};

class Errors {
 public:
  Errors() = default;
  Errors(std::array<bool, 24> errors);

  bool any() const;
  bool violated(ErrorType error) const;
  std::string name(ErrorType error) const;
  std::string namesViolated() const;

 private:
  std::array<bool, 24> errors_;
};

std::ostream& operator<<(std::ostream& ostream, const franka::Errors& errors);

}  // namespace franka