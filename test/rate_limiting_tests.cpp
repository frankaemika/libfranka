// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>

#include <franka/rate_limiting.h>

using namespace franka;

std::array<double, 7> kNoLimit{
    {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
     std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
     std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
     std::numeric_limits<double>::max()}};

std::array<double, 7> integrateOneSample(std::array<double, 7> last_value,
                                         std::array<double, 7> derivative,
                                         double delta_t) {
  std::array<double, 7> result{};
  for (size_t i = 0; i < 7; i++) {
    result[i] = last_value[i] + derivative[i] * delta_t;
  }
  return result;
}

std::array<double, 7> differentiateOneSample(std::array<double, 7> value,
                                             std::array<double, 7> last_value,
                                             double delta_t) {
  std::array<double, 7> result{};
  for (size_t i = 0; i < 7; i++) {
    result[i] = (value[i] - last_value[i]) / delta_t;
  }
  return result;
}

bool violatesLimits(double desired_value, double max_value) {
  return std::abs(desired_value) > max_value;
}

bool violatesLimits(std::array<double, 7> desired_values, std::array<double, 7> max_values) {
  bool violates_limits = false;
  for (size_t i = 0; i < 7 && !violates_limits; i++) {
    violates_limits = violates_limits || violatesLimits(desired_values[i], max_values[i]);
  }
  return violates_limits;
}

bool violatesRateLimits(std::array<double, 7> max_derivatives,
                        std::array<double, 7> desired_values,
                        std::array<double, 7> last_desired_values) {
  return violatesLimits(differentiateOneSample(desired_values, last_desired_values, kDeltaT),
                        max_derivatives);
}

bool violatesRateLimits(std::array<double, 7> max_values,
                        std::array<double, 7> max_derivatives,
                        std::array<double, 7> max_dderivatives,
                        std::array<double, 7> desired_values,
                        std::array<double, 7> last_desired_values,
                        std::array<double, 7> last_derivative_values) {
  std::array<double, 7> desired_derivatives =
      differentiateOneSample(desired_values, last_desired_values, kDeltaT);

  return violatesLimits(desired_values, max_values) ||
         violatesRateLimits(max_derivatives, desired_values, last_desired_values) ||
         violatesRateLimits(max_dderivatives, desired_derivatives, last_derivative_values);
}

std::array<double, 7> generateValuesIntoLimits(std::array<double, 7> last_desired_values,
                                               std::array<double, 7> max_derivatives,
                                               double eps) {
  std::array<double, 7> desired_value{};
  for (size_t i = 0; i < 7; i++) {
    // Make sure that the integration yields a value into limits
    desired_value[i] =
        last_desired_values[i] +
        (max_derivatives[i] - std::min(std::max(std::abs(eps), 0.0), 2.0 * max_derivatives[i])) *
            kDeltaT;
  }
  return desired_value;
}

std::array<double, 7> generateValuesOutsideLimits(std::array<double, 7> last_desired_values,
                                                  std::array<double, 7> max_derivatives,
                                                  double eps,
                                                  double delta_t) {
  std::array<double, 7> desired_value{};
  for (size_t i = 0; i < 7; i++) {
    // Make sure that diff yields a value outside limits
    desired_value[i] = last_desired_values[i] +
                       (max_derivatives[i] + std::max(std::abs(eps), kLimitEps)) * delta_t;
  }
  return desired_value;
}

TEST(RateLimiting, MaxDerivative) {
  std::array<double, 7> max_derivatives{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  std::array<double, 7> last_desired_values{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate
  std::array<double, 7> values_into_limits =
      generateValuesIntoLimits(last_desired_values, max_derivatives, eps);
  EXPECT_FALSE(violatesRateLimits(max_derivatives, values_into_limits, last_desired_values));
  EXPECT_EQ(values_into_limits,
            limitRate(max_derivatives, values_into_limits, last_desired_values));

  // Desired values are outside limits and limited after limitRate
  std::array<double, 7> values_outside_limits =
      generateValuesOutsideLimits(last_desired_values, max_derivatives, eps, kDeltaT);
  std::array<double, 7> limited_values =
      limitRate(max_derivatives, values_outside_limits, last_desired_values);
  EXPECT_TRUE(violatesRateLimits(max_derivatives, values_outside_limits, last_desired_values));
  EXPECT_NE(values_outside_limits, limited_values);
  EXPECT_FALSE(violatesRateLimits(max_derivatives, limited_values, last_desired_values));
}

TEST(RateLimiting, JointVelocity) {
  std::array<double, 7> last_desired_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_desired_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> max_acceleration{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  std::array<double, 7> max_jerk{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (jerk)
  std::array<double, 7> joint_velocity_into_limits = integrateOneSample(
      last_desired_velocity, generateValuesIntoLimits(last_desired_acceleration, max_jerk, eps),
      kDeltaT);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, kNoLimit, max_jerk, joint_velocity_into_limits,
                                  last_desired_velocity, last_desired_acceleration));
  EXPECT_EQ(joint_velocity_into_limits,
            limitRate(kNoLimit, kNoLimit, max_jerk, joint_velocity_into_limits,
                      last_desired_velocity, last_desired_acceleration));

  // Desired values are into limits and unchanged after limitRate (acceleration)
  joint_velocity_into_limits =
      generateValuesIntoLimits(last_desired_velocity, max_acceleration, eps);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, max_acceleration, kNoLimit, joint_velocity_into_limits,
                                  last_desired_velocity, last_desired_acceleration));
  EXPECT_EQ(joint_velocity_into_limits,
            limitRate(kNoLimit, max_acceleration, kNoLimit, joint_velocity_into_limits,
                      last_desired_velocity, last_desired_acceleration));

  // Desired values are outside limits and limited after limitRate (jerk)
  std::array<double, 7> joint_velocity_outside_limits = integrateOneSample(
      last_desired_velocity,
      generateValuesOutsideLimits(last_desired_acceleration, max_jerk, eps, kDeltaT), kDeltaT);
  std::array<double, 7> limited_joint_velocity =
      limitRate(kNoLimit, kNoLimit, max_jerk, joint_velocity_outside_limits, last_desired_velocity,
                last_desired_acceleration);
  EXPECT_TRUE(violatesRateLimits(kNoLimit, kNoLimit, max_jerk, joint_velocity_outside_limits,
                                 last_desired_velocity, last_desired_acceleration));
  EXPECT_NE(joint_velocity_outside_limits, limited_joint_velocity);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, kNoLimit, max_jerk, limited_joint_velocity,
                                  last_desired_velocity, last_desired_acceleration));

  // Desired values are outside limits and limited after limitRate (acceleration)
  joint_velocity_outside_limits =
      generateValuesOutsideLimits(last_desired_velocity, max_acceleration, eps, kDeltaT);
  limited_joint_velocity =
      limitRate(kNoLimit, max_acceleration, kNoLimit, joint_velocity_outside_limits,
                last_desired_velocity, last_desired_acceleration);
  EXPECT_TRUE(violatesRateLimits(kNoLimit, max_acceleration, kNoLimit,
                                 joint_velocity_outside_limits, last_desired_velocity,
                                 last_desired_acceleration));
  EXPECT_NE(joint_velocity_outside_limits, limited_joint_velocity);
  EXPECT_FALSE(violatesRateLimits(kNoLimit, max_acceleration, kNoLimit, limited_joint_velocity,
                                  last_desired_velocity, last_desired_acceleration));
}

TEST(RateLimiting, JointPosition) {
  std::array<double, 7> last_desired_position{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_desired_velocity{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> last_desired_acceleration{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> max_acceleration{{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  std::array<double, 7> max_jerk{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
  double eps{1e-2};

  // Desired values are into limits and unchanged after limitRate (jerk)
  std::array<double, 7> joint_position_into_limits = integrateOneSample(
      last_desired_position,
      integrateOneSample(last_desired_velocity,
                         generateValuesIntoLimits(last_desired_acceleration, max_jerk, eps),
                         kDeltaT),
      kDeltaT);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, kNoLimit, max_jerk,
      differentiateOneSample(joint_position_into_limits, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));
  EXPECT_EQ(joint_position_into_limits,
            limitRate(kNoLimit, kNoLimit, max_jerk, joint_position_into_limits,
                      last_desired_position, last_desired_velocity, last_desired_acceleration));

  // Desired values are into limits and unchanged after limitRate (acceleration)
  joint_position_into_limits = integrateOneSample(
      last_desired_position, generateValuesIntoLimits(last_desired_velocity, max_acceleration, eps),
      kDeltaT);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, max_acceleration, kNoLimit,
      differentiateOneSample(joint_position_into_limits, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));
  EXPECT_EQ(joint_position_into_limits,
            limitRate(kNoLimit, max_acceleration, kNoLimit, joint_position_into_limits,
                      last_desired_position, last_desired_velocity, last_desired_acceleration));

  // Desired values are outside limits and limited after limitRate (jerk)
  std::array<double, 7> joint_position_outside_limits = integrateOneSample(
      last_desired_position,
      integrateOneSample(
          last_desired_velocity,
          generateValuesOutsideLimits(last_desired_acceleration, max_jerk, eps, kDeltaT), kDeltaT),
      kDeltaT);
  std::array<double, 7> limited_joint_position =
      limitRate(kNoLimit, kNoLimit, max_jerk, joint_position_outside_limits, last_desired_position,
                last_desired_velocity, last_desired_acceleration);
  EXPECT_TRUE(violatesRateLimits(
      kNoLimit, kNoLimit, max_jerk,
      differentiateOneSample(joint_position_outside_limits, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));
  EXPECT_NE(joint_position_outside_limits, limited_joint_position);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, kNoLimit, max_jerk,
      differentiateOneSample(limited_joint_position, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));

  // Desired values outside limit (acceleration)
  joint_position_outside_limits = integrateOneSample(
      last_desired_position,
      generateValuesOutsideLimits(last_desired_velocity, max_acceleration, eps, kDeltaT), kDeltaT);
  limited_joint_position =
      limitRate(kNoLimit, max_acceleration, kNoLimit, joint_position_outside_limits,
                last_desired_position, last_desired_velocity, last_desired_acceleration);
  EXPECT_TRUE(violatesRateLimits(
      kNoLimit, max_acceleration, kNoLimit,
      differentiateOneSample(joint_position_outside_limits, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));
  EXPECT_NE(joint_position_outside_limits, limited_joint_position);
  EXPECT_FALSE(violatesRateLimits(
      kNoLimit, max_acceleration, kNoLimit,
      differentiateOneSample(limited_joint_position, last_desired_position, kDeltaT),
      last_desired_velocity, last_desired_acceleration));
}

TEST(RateLimiting, CartesianVelocity) {
  std::array<double, 6> O_dP_EE_d{0.901, 0.801, 0.701, 0.601, 0.501, 0.401};
  std::array<double, 6> last_O_dP_EE_d{0.9, 0.8, 0.7, 0.6, 0.5, 0.4};
  std::array<double, 6> last_O_ddP_EE_d{0.9, 0.8, 0.7, 0.6, 0.5, 0.4};

  // Desired values are not limited
  std::array<double, 6> expected = O_dP_EE_d;
  std::array<double, 6> limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, O_dP_EE_d,
                last_O_dP_EE_d, last_O_ddP_EE_d);
  EXPECT_EQ(expected, limited_desired_values);

  // Desired values are limited
  O_dP_EE_d = std::array<double, 6>{0.99, 0.88, 0.77, 0.66, 0.55, 0.44};
  last_O_ddP_EE_d = std::array<double, 6>{0.99, 0.88, 0.77, 0.66, 0.55, 0.44};
  expected = std::array<double, 6>{0.90924, 0.80821, 0.70718, 0.61880, 0.51566, 0.41253};
  limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, O_dP_EE_d,
                last_O_dP_EE_d, last_O_ddP_EE_d);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }
}

// Values are taken from the output of the generate_cartesian_pose_motion example
TEST(RateLimiting, CartesianPose) {
  std::array<double, 16> O_T_EE_d_initial{
      0.707107,     -0.707107,   -3.44281e-07, 0, -0.707107, -0.707107,   1.70222e-07, 0,
      -3.63809e-07, 1.23078e-07, -1,           0, 0.546792,  3.20732e-09, 0.470414,    1};
  std::array<double, 16> last_O_T_EE_d{
      0.707107,     -0.707107,   -3.44281e-07, 0, -0.707107, -0.707107,   1.70223e-07, 0,
      -3.63809e-07, 1.23078e-07, -1,           0, 0.546704,  3.20727e-09, 0.47053,     1};
  std::array<double, 6> last_O_dP_EE_d{0.0875399,    0,          -0.11641, -2.45194e-20,
                                       -1.93244e-20, 7.85046e-14};
  std::array<double, 6> last_O_ddP_EE_d{0.0875399,    0,          -0.11641, -2.45194e-20,
                                        -1.93244e-20, 7.85046e-14};

  // Desired values are not limited
  std::array<double, 16> O_T_EE_d = O_T_EE_d_initial;
  std::array<double, 16> expected = O_T_EE_d;
  std::array<double, 16> limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, O_T_EE_d,
                last_O_T_EE_d, last_O_dP_EE_d, last_O_ddP_EE_d);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }

  // Desired values are limited, O_T_EE_d is 4 time steps further:
  O_T_EE_d = std::array<double, 16>{
      0.707107,     -0.707107,   -3.44281e-07, 0, -0.707107, -0.707107,   1.70222e-07, 0,
      -3.63809e-07, 1.23078e-07, -1,           0, 0.547141,  3.20732e-09, 0.469948,    1};
  expected = O_T_EE_d_initial;
  limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxTranslationalJerk,
                kMaxRotationalVelocity, kMaxRotationalAcceleration, kMaxRotationalJerk, O_T_EE_d,
                last_O_T_EE_d, last_O_dP_EE_d, last_O_ddP_EE_d);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }
}
