#include <gmock/gmock.h>

#include <franka/model.h>

#include "helpers.h"
#include "mock_server.h"
#include "model_library_interface.h"

using ::testing::_;
using ::testing::WithArgs;
using ::testing::Invoke;

struct MockModel : public ModelLibraryInterface {
  MOCK_METHOD3(O_T_J1, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J2, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J3, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J4, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J5, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J6, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J7, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J8, void(const double*, const double*, double*));
  MOCK_METHOD3(O_T_J9, void(const double*, const double*, double*));
  MOCK_METHOD5(M_NE, void(const double*, const double*, double,
                          const double*, double*));
  MOCK_METHOD6(c_NE, void(const double*, const double*, const double*,
                          double, const double*, double*));
  MOCK_METHOD5(g_NE, void(const double*, const double*, double,
                          const double*, double*));
};

struct Model : public ::testing::Test {
  Model() {
    MockServer server;
    server.spinOnce();
    robot.reset(new franka::Robot("127.0.0.1"));

    model_library_interface = nullptr;
  }

  std::unique_ptr<franka::Robot> robot;
};

TEST_F(Model, CanCreateModel) {
  EXPECT_NO_THROW(franka::Model model(*robot));
}

TEST_F(Model, CanGetMassMatrix) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 9> load_inertia{0, 1, 2, 3, 4, 5, 6, 7, 8};
  double load_mass = 0.75;
  std::array<double, 3> F_x_Cload{9, 10, 11};

  MockModel mock;
  EXPECT_CALL(mock, M_NE(robot_state.q.data(),
                         load_inertia.data(),
                         load_mass,
                         F_x_Cload.data(),
                         _))
    .WillOnce(WithArgs<4>(Invoke([=](double* output) {
      for (size_t i = 0; i < 49; i++) {
        output[i] = i;
      }
    })));

  model_library_interface = &mock;

  franka::Model model(*robot);
  auto matrix = model.mass(robot_state, load_inertia, load_mass, F_x_Cload);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetCoriolisVector) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 9> load_inertia{0, 1, 2, 3, 4, 5, 6, 7, 8};
  double load_mass = 0.75;
  std::array<double, 3> F_x_Cload{9, 10, 11};
  std::array<double, 7> expected_vector{12, 13, 14, 15, 16, 17, 18};

  MockModel mock;
  EXPECT_CALL(mock, c_NE(robot_state.q.data(),
                         robot_state.dq.data(),
                         load_inertia.data(),
                         load_mass,
                         F_x_Cload.data(),
                         _))
    .WillOnce(WithArgs<5>(Invoke([=](double* output) {
      std::copy(expected_vector.cbegin(), expected_vector.cend(), output);
    })));

  model_library_interface = &mock;

  franka::Model model(*robot);
  auto vector = model.coriolis(robot_state, load_inertia, load_mass, F_x_Cload);
  EXPECT_EQ(expected_vector, vector);
}

TEST_F(Model, CanGetGravity) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  double load_mass = 0.75;
  std::array<double, 3> F_x_Cload{1, 2, 3};
  std::array<double, 3> gravity_earth{4, 5, 6};

  MockModel mock;
  EXPECT_CALL(mock, g_NE(robot_state.q.data(),
                         gravity_earth.data(),
                         load_mass,
                         F_x_Cload.data(),
                         _))
    .WillOnce(WithArgs<4>(Invoke([=](double* output) {
      for (size_t i = 0; i < 7; i++) {
        output[i] = i;
      }
    })));

  model_library_interface = &mock;

  franka::Model model(*robot);
  auto matrix = model.gravity(robot_state, load_mass, F_x_Cload, gravity_earth);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetJointPoses) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  std::array<double, 16> expected_pose{{0, 1, 2, 3, 4, 5, 6, 7, 8,
                                       9, 10, 11, 12, 13, 14, 15}};

  MockModel mock;
  EXPECT_CALL(mock, O_T_J1(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J2(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J3(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J4(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J5(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J6(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J7(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J8(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));
  EXPECT_CALL(mock, O_T_J9(robot_state.q.data(),
                           robot_state.O_T_EE.data(),
                           _))
    .WillOnce(WithArgs<2>(Invoke([=](double* output) {
      std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
    })));

  model_library_interface = &mock;

  franka::Model model(*robot);
  for (franka::Frame joint = franka::Frame::kJoint1;
       joint <= franka::Frame::kEndEffector;
       joint = static_cast<franka::Frame>(joint + 1)) {
    auto pose = model.jointPose(joint, robot_state);
    EXPECT_EQ(expected_pose, pose);
  }
}
