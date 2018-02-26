// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>

#include <franka/rate_limiting.h>

using namespace franka;

TEST(RateLimiting, MaxDerivative) {
  std::array<double, 7> max_derivatives{1000, 1000, 1000, 1000, 1000, 1000, 1000};
  std::array<double, 7> desired_values{11, 10, 9, 8, 7, 6, 5};
  std::array<double, 7> last_desired_values{10, 9, 8, 7, 6, 5, 4};

  // Desired values are not limited
  std::array<double, 7> expected = desired_values;
  std::array<double, 7> limited_desired_values =
      limitRate(max_derivatives, desired_values, last_desired_values);
  EXPECT_EQ(expected, limited_desired_values);

  // Desired values are limited
  max_derivatives = std::array<double, 7>{500, 500, 500, 500, 500, 500, 500};
  expected = std::array<double, 7>{10.5, 9.5, 8.5, 7.5, 6.5, 5.5, 4.5};
  limited_desired_values = limitRate(max_derivatives, desired_values, last_desired_values);
  EXPECT_EQ(expected, limited_desired_values);
}

TEST(RateLimiting, JointVelocity) {
  std::array<double, 7> desired_velocities{0.905, 0.805, 0.705, 0.605, 0.505, 0.405, 0.305};
  std::array<double, 7> last_desired_velocities{0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};

  // Desired values are not limited
  std::array<double, 7> expected = desired_velocities;
  std::array<double, 7> limited_desired_values = limitRate(
      kMaxJointVelocity, kMaxJointAcceleration, desired_velocities, last_desired_velocities);
  EXPECT_EQ(expected, limited_desired_values);

  // Desired values are limited
  desired_velocities = std::array<double, 7>{1.95, 1.82, 1.72, 1.62, 1.52, 1.42, 1.32};
  last_desired_velocities = std::array<double, 7>{0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
  expected = std::array<double, 7>{0.9165, 0.80825, 0.71375, 0.61375, 0.5165, 0.422, 0.322};
  limited_desired_values = limitRate(kMaxJointVelocity, kMaxJointAcceleration, desired_velocities,
                                     last_desired_velocities);

  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }
}

TEST(RateLimiting, JointPosition) {
  std::array<double, 7> desired_positions{0.901, 0.801, 0.701, 0.601, 0.501, 0.401, 0.301};
  std::array<double, 7> last_desired_positions{0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
  std::array<double, 7> last_desired_velocities{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // Desired values are not limited
  std::array<double, 7> expected = desired_positions;
  std::array<double, 7> limited_desired_values =
      limitRate(kMaxJointVelocity, kMaxJointAcceleration, desired_positions, last_desired_positions,
                last_desired_velocities);
  EXPECT_EQ(expected, limited_desired_values);

  // Desired values are limited
  last_desired_velocities = std::array<double, 7>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  expected = std::array<double, 7>{0.9001, 0.8001, 0.7001, 0.6001, 0.5001, 0.4001, 0.3001};
  limited_desired_values = limitRate(kMaxJointVelocity, kMaxJointAcceleration, desired_positions,
                                     last_desired_positions, last_desired_velocities);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }
}

TEST(RateLimiting, CartesianVelocity) {
  std::array<double, 6> O_dP_EE_d{0.901, 0.801, 0.701, 0.601, 0.501, 0.401};
  std::array<double, 6> last_O_dP_EE_d{0.9, 0.8, 0.7, 0.6, 0.5, 0.4};

  // Desired values are not limited
  std::array<double, 6> expected = O_dP_EE_d;
  std::array<double, 6> limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxRotationalVelocity,
                kMaxRotationalAcceleration, O_dP_EE_d, last_O_dP_EE_d);
  EXPECT_EQ(expected, limited_desired_values);

  // Desired values are limited
  O_dP_EE_d = std::array<double, 6>{0.99, 0.88, 0.77, 0.66, 0.55, 0.44};
  expected = std::array<double, 6>{0.90924, 0.80821, 0.70718, 0.61880, 0.51566, 0.41253};
  limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxRotationalVelocity,
                kMaxRotationalAcceleration, O_dP_EE_d, last_O_dP_EE_d);
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

  // Desired values are not limited
  std::array<double, 16> O_T_EE_d = O_T_EE_d_initial;
  std::array<double, 16> expected = O_T_EE_d;
  std::array<double, 16> limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxRotationalVelocity,
                kMaxRotationalAcceleration, O_T_EE_d, last_O_T_EE_d, last_O_dP_EE_d);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }

  // Desired values are limited, O_T_EE_d is 4 time steps further:
  O_T_EE_d = std::array<double, 16>{
      0.707107,     -0.707107,   -3.44281e-07, 0, -0.707107, -0.707107,   1.70222e-07, 0,
      -3.63809e-07, 1.23078e-07, -1,           0, 0.547141,  3.20732e-09, 0.469948,    1};
  expected = O_T_EE_d_initial;
  limited_desired_values =
      limitRate(kMaxTranslationalVelocity, kMaxTranslationalAcceleration, kMaxRotationalVelocity,
                kMaxRotationalAcceleration, O_T_EE_d, last_O_T_EE_d, last_O_dP_EE_d);
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected[i], limited_desired_values[i], 1e-4);
  }
}
