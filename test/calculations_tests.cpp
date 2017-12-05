// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <gtest/gtest.h>

#include "load_calculations.h"

TEST(CalculationTest, CombineCenterOfMassZeroMassInput) {
  double m_ee = 0;
  double m_load = 0;
  std::array<double, 3> F_x_Ctotal =
      franka::combineCenterOfMass(m_ee, std::array<double, 3>{}, m_load, std::array<double, 3>{});

  std::array<double, 3> expected{};
  EXPECT_EQ(expected, F_x_Ctotal);
}

TEST(CalculationTest, CombineCenterOfMassEEWithZeroLoad) {
  double m_ee = 0.73;
  std::array<double, 3> F_x_Cee{-0.01, 0, 0.03};
  double m_load = 0.0;
  std::array<double, 3> F_x_Cload{};

  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  EXPECT_EQ(F_x_Cee, F_x_Ctotal);
}

TEST(CalculationTest, CombineCenterOfMassLoadWithZeroEE) {
  double m_ee = 0.0;
  std::array<double, 3> F_x_Cee{};
  double m_load = 0.73;
  std::array<double, 3> F_x_Cload{-0.01, 0, 0.03};
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  EXPECT_EQ(F_x_Cload, F_x_Ctotal);
}

TEST(CalculationTest, CombineCenterOfMassEEWithLoad) {
  double m_ee = 0.73;
  std::array<double, 3> F_x_Cee{-0.01, 0, -0.03};
  double m_load = 0.5;
  std::array<double, 3> F_x_Cload{0.01, -0.2, 0.03};
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  std::array<double, 3> expected{-0.00186991869918699, -0.08130081300813009, -0.00560975609756098};
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(expected[i], F_x_Ctotal[i], 1e-17);
  }
}

TEST(CalculationTest, CombineInertiaTensorZeroMassInput) {
  double m_ee = 0.0;
  std::array<double, 3> F_x_Cee{-0.01, 0, -0.03};
  std::array<double, 9> I_ee{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017};
  double m_load = 0.0;
  std::array<double, 3> F_x_Cload{0.01, -0.2, 0.03};
  std::array<double, 9> I_load{};
  double m_total = m_ee + m_load;
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  std::array<double, 9> I_total = franka::combineInertiaTensor(
      m_ee, F_x_Cee, I_ee, m_load, F_x_Cload, I_load, m_total, F_x_Ctotal);

  std::array<double, 9> expected{};
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(expected[i], I_total[i], 1e-17);
  }
}

TEST(CalculationTest, CombineInertiaTensorLoadWithZeroEE) {
  double m_ee = 0.0;
  std::array<double, 3> F_x_Cee{-0.01, 0, -0.03};
  std::array<double, 9> I_ee{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017};
  double m_load = 0.5;
  std::array<double, 3> F_x_Cload{0.01, -0.2, 0.03};
  std::array<double, 9> I_load{0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3};
  double m_total = m_ee + m_load;
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  std::array<double, 9> I_total = franka::combineInertiaTensor(
      m_ee, F_x_Cee, I_ee, m_load, F_x_Cload, I_load, m_total, F_x_Ctotal);

  std::array<double, 9> expected = I_load;
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(expected[i], I_total[i], 1e-17);
  }
}

TEST(CalculationTest, CombineInertiaTensorEEWithZeroLoad) {
  double m_ee = 0.73;
  std::array<double, 3> F_x_Cee{-0.01, 0, -0.03};
  std::array<double, 9> I_ee{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017};
  double m_load = 0.0;
  std::array<double, 3> F_x_Cload{0.01, -0.2, 0.03};
  std::array<double, 9> I_load{0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3};
  double m_total = m_ee + m_load;
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  std::array<double, 9> I_total = franka::combineInertiaTensor(
      m_ee, F_x_Cee, I_ee, m_load, F_x_Cload, I_load, m_total, F_x_Ctotal);

  std::array<double, 9> expected = I_ee;
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(expected[i], I_total[i], 1e-17);
  }
}

TEST(CalculationTest, CombineInertiaTensorEEWithLoad) {
  double m_ee = 0.73;
  std::array<double, 3> F_x_Cee{-0.01, 0, -0.03};
  std::array<double, 9> I_ee{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017};
  double m_load = 0.5;
  std::array<double, 3> F_x_Cload{0.01, -0.2, 0.03};
  std::array<double, 9> I_load{0.001, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.3};
  double m_total = m_ee + m_load;
  std::array<double, 3> F_x_Ctotal = franka::combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);

  std::array<double, 9> I_total = franka::combineInertiaTensor(
      m_ee, F_x_Cee, I_ee, m_load, F_x_Cload, I_load, m_total, F_x_Ctotal);

  std::array<double, 9> expected{1.49382113821138e-02,  1.18699186991870e-03, -3.56097560975610e-04,
                                 1.18699186991870e-03,  2.36869918699187e-02, 3.56097560975610e-03,
                                 -3.56097560975610e-04, 3.56097560975610e-03, 3.13688617886179e-01};
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(expected[i], I_total[i], 1e-14);
  }
}