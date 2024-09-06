// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "franka/robot_model.h"
#include "gtest/gtest.h"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>

constexpr double kEps = 1e-5;
constexpr double kDeltaCoriolis = 0.2;
constexpr double kDeltaInertia = 0.1;
constexpr double kDeltaGravity = 0.2;

constexpr double kDeltaGravityDot = 0.4;
constexpr double kDeltaInertiaDot = 0.03;
constexpr double kDeltaCoriolisDot = 0.3;

std::string current_file_path = __FILE__;
std::string urdf_path =
    current_file_path.substr(0, current_file_path.find_last_of("/\\") + 1) + "fr3.urdf";

std::string readFileToString(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

class RobotModelTest : public ::testing::Test {
 protected:
  std::unique_ptr<franka::RobotModel> model;
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> dq = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> unit_dq = {1, 1, 1, 1, 1, 1, 1};
  std::array<double, 9> i_total = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::array<double, 3> f_x_ctotal = {0, 0, 0};
  double m_total = 0;
  std::array<double, 3> g_earth = {0, 0, -9.81};
  std::array<double, 7> default_joint_configuration{0, 0, 0, -0.75 * M_PI, 0, 0.75 * M_PI, 0};
  std::array<double, 7> moved_joint_configuration{0.0010, 0.0010, 0.0010, -2.3552,
                                                  0.0010, 2.3572, 0.0010};

  // Franka end-effector mechanical properties
  std::array<double, 9> ee_i_total = {0.001, 0, 0, 0, 0.0025, 0, 0, 0, 0.0017};
  std::array<double, 3> ee_f_x_ctotal = {0.01, 0, 0.03};
  double ee_m_total = 0.73;

  RobotModelTest() {
    auto urdf_string = readFileToString(urdf_path);
    model = std::make_unique<franka::RobotModel>(urdf_string);
  }
};

TEST_F(RobotModelTest, TestCoriolis) {
  std::array<double, 7> c_ne;

  model->coriolis(q, dq, i_total, m_total, f_x_ctotal, c_ne);

  for (double i : c_ne) {
    ASSERT_EQ(i, 0.0);
  }
}

TEST_F(RobotModelTest, TestGravity) {
  std::array<double, 3> g_earth = {0, 0, -9.81};
  std::array<double, 7> g_ne;

  model->gravity(q, g_earth, m_total, f_x_ctotal, g_ne);

  Eigen::Matrix<double, 7, 1> expected_gravity;
  expected_gravity << 0, -3.52387, 0, -3.44254, 0, 1.63362, -2.47128e-17;

  for (int i = 0; i < expected_gravity.rows(); i++) {
    ASSERT_NEAR(g_ne[i], expected_gravity(i), kEps);
  }
}

TEST_F(RobotModelTest, TestMass) {
  std::array<double, 49> m_ne;

  model->mass(q, i_total, m_total, f_x_ctotal, m_ne);

  Eigen::Matrix<double, 7, 7> expected_mass_matrix;
  expected_mass_matrix << 0.0875133, -0.0405634, 0.0608543, 0.0122244, 0.0237562, -0.00144562,
      -0.000309373, -0.0405634, 2.66309, -0.0464397, -1.10219, -0.0415206, 0.0234914, -0.00387413,
      0.0608543, -0.0464397, 0.0608543, 0.0122244, 0.0237562, -0.00144562, -0.000309373, 0.0122244,
      -1.10219, 0.0122244, 0.591968, 0.0158788, -0.0168746, 0.00216462, 0.0237562, -0.0415206,
      0.0237562, 0.0158788, 0.0237562, -0.00144562, -0.000309373, -0.00144562, 0.0234914,
      -0.00144562, -0.0168746, -0.00144562, 0.0215993, 8.72329e-05, -0.000309373, -0.00387413,
      -0.000309373, 0.00216462, -0.000309373, 8.72329e-05, 5.97777e-05;

  for (int i = 0; i < expected_mass_matrix.size(); i++) {
    ASSERT_NEAR(m_ne[i], expected_mass_matrix(i), kEps);
  }
}

TEST_F(RobotModelTest, TestCoriolisWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 7> c_ne;

  model->coriolis(q, unit_dq, ee_i_total, ee_m_total, ee_f_x_ctotal, c_ne);

  Eigen::Matrix<double, 7, 1> expected_coriolis;
  expected_coriolis << 0.223409, -2.07416, 0.224134, 1.05834, 0.0876554, -0.222425, -0.00102552;
  ;

  for (int i = 0; i < expected_coriolis.rows(); i++) {
    ASSERT_NEAR(c_ne[i], expected_coriolis(i), kEps);
  }
}

TEST_F(RobotModelTest, TestGravityWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 7> g_ne;

  model->gravity(q, g_earth, ee_m_total, ee_f_x_ctotal, g_ne);

  Eigen::Matrix<double, 7, 1> expected_gravity;
  expected_gravity << 0, -4.22568, 0, -3.33154, 0, 2.33543, -8.83179e-17;
  ;

  for (int i = 0; i < expected_gravity.rows(); i++) {
    ASSERT_NEAR(g_ne[i], expected_gravity(i), kEps);
  }
}

TEST_F(RobotModelTest, TestMassWithAddedFrankaEndEffectorInertiaToLastLink) {
  std::array<double, 49> m_ne;

  model->mass(q, ee_i_total, ee_m_total, ee_f_x_ctotal, m_ne);

  Eigen::Matrix<double, 7, 7> expected_mass_matrix;
  expected_mass_matrix << 0.0962242, -0.0405634, 0.0695652, 0.0122244, 0.0324671, -0.00144562,
      -0.00272477, -0.0405634, 2.90398, -0.0464397, -1.20732, -0.0415206, 0.0702861, -0.00387413,
      0.0695652, -0.0464397, 0.0695652, 0.0122244, 0.0324671, -0.00144562, -0.00272477, 0.0122244,
      -1.20732, 0.0122244, 0.63918, 0.0158788, -0.0379682, 0.00216462, 0.0324671, -0.0415206,
      0.0324671, 0.0158788, 0.0324671, -0.00144562, -0.00272477, -0.00144562, 0.0702861,
      -0.00144562, -0.0379682, -0.00144562, 0.0448116, 8.72329e-05, -0.00272477, -0.00387413,
      -0.00272477, 0.00216462, -0.00272477, 8.72329e-05, 0.00183278;

  for (int i = 0; i < expected_mass_matrix.size(); i++) {
    ASSERT_NEAR(m_ne[i], expected_mass_matrix(i), kEps);
  }
}

TEST_F(RobotModelTest, givenNoLoad_whenComputingGravity_thenCloseToBaseline) {
  std::array<double, 7> test_gravity;

  auto expected_gravity = std::array<double, 7>{0, -24.5858, 0, 17.6880, 0.5095, 1.6428, 0};

  model->gravity(default_joint_configuration, g_earth, m_total, f_x_ctotal, test_gravity);

  for (size_t i = 0; i < test_gravity.size(); ++i) {
    ASSERT_NEAR(test_gravity[i], expected_gravity[i], kDeltaGravity) << "Joint " << i;
  }
}

TEST_F(RobotModelTest, givenLoad_whenComputingGravity_thenCloseToBaseline) {
  std::array<double, 7> test_gravity;

  auto expected_gravity = std::array<double, 7>{0, -28.2417, 0, 20.7531, 0.5095, 2.3446, 0};
  model->gravity(moved_joint_configuration, g_earth, ee_m_total, ee_f_x_ctotal, test_gravity);

  for (size_t i = 0; i < test_gravity.size(); ++i) {
    ASSERT_NEAR(test_gravity[i], expected_gravity[i], kDeltaGravity) << "Joint " << i;
  }
}

TEST_F(RobotModelTest, givenNonZeroJointVelocity_whenComputingCoriolis_thenNonZeroCoriolis) {
  auto expected_coriolis =
      std::array<double, 7>{2.4562, -0.8199, 2.4532, -2.3450, -0.1492, -0.1801, 0.0164};

  std::array<double, 7> test_coriolis;
  model->coriolis(default_joint_configuration, unit_dq, i_total, m_total, f_x_ctotal,
                  test_coriolis);

  for (size_t i = 0; i < test_coriolis.size(); ++i) {
    ASSERT_NEAR(test_coriolis[i], expected_coriolis[i], kDeltaCoriolis);
  }
}

TEST_F(RobotModelTest,
       givenNonZeroJointVelocityWithLoad_whenComputingCoriolis_thenCloseToBaseline) {
  std::array<double, 7> test_coriolis;

  auto expected_coriolis =
      std::array<double, 7>{3.1468, -0.6225, 3.1439, -3.1063, -0.0658, -0.5416, 0.0089};

  model->coriolis(moved_joint_configuration, unit_dq, ee_i_total, ee_m_total, ee_f_x_ctotal,
                  test_coriolis);
  for (size_t i = 0; i < test_coriolis.size(); ++i) {
    ASSERT_NEAR(test_coriolis[i], expected_coriolis[i], kDeltaCoriolis);
  }
}

TEST_F(RobotModelTest, givenWithLoad_whenComputingInertia_thenCloseToBaseline) {
  std::array<double, 7 * 7> test_inertia;

  auto expected_inertia = std::array<double, 7 * 7>{
      1.1749,  -0.0085, 1.1660,  -0.0247, -0.0261, -0.0033, -0.0057, -0.0085, 1.7281,  0.0046,
      -0.8304, -0.0142, -0.1209, -0.0005, 1.1660,  0.0046,  1.1660,  -0.0247, -0.0261, -0.0033,
      -0.0057, -0.0247, -0.8304, -0.0247, 1.0399,  0.0263,  0.1526,  -0.0023, -0.0261, -0.0142,
      -0.0261, 0.0263,  0.0112,  0.0042,  0.0012,  -0.0033, -0.1209, -0.0033, 0.1526,  0.0042,
      0.0390,  -0.0004, -0.0057, -0.0005, -0.0057, -0.0023, 0.0012,  -0.0004, 0.0019};

  model->mass(moved_joint_configuration, ee_i_total, ee_m_total, ee_f_x_ctotal, test_inertia);

  for (size_t i = 0; i < test_inertia.size(); ++i) {
    ASSERT_NEAR(test_inertia[i], expected_inertia[i], kDeltaInertia);
  }
}

TEST_F(RobotModelTest, givenNoLoad_whenComputingInertia_thenCloseToBaseline) {
  std::array<double, 7 * 7> test_inertia;

  auto expected_inertia = std::array<double, 7 * 7>{
      0.9831,  -0.0078, 0.9742,  -0.0254, -0.0351, -0.0040, -0.0004, -0.0078, 1.5390,  0.0053,
      -0.6641, -0.0147, -0.0828, -0.0012, 0.9742,  0.0053,  0.9742,  -0.0254, -0.0351, -0.0040,
      -0.0004, -0.0254, -0.6641, -0.0254, 0.8185,  0.0268,  0.0888,  -0.0016, -0.0351, -0.0147,
      -0.0351, 0.0268,  0.0114,  0.0047,  0.0002,  -0.0040, -0.0828, -0.0040, 0.0888,  0.0047,
      0.0202,  0.0003,  -0.0004, -0.0012, -0.0004, -0.0016, 0.0002,  0.0003,  0.0002};

  model->mass(default_joint_configuration, i_total, m_total, f_x_ctotal, test_inertia);
  for (size_t i = 0; i < test_inertia.size(); ++i) {
    ASSERT_NEAR(test_inertia[i], expected_inertia[i], kDeltaInertia);
  }
}
