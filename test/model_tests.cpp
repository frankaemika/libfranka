// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <fstream>
#include <memory>

#include <gmock/gmock.h>
#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <research_interface/robot/service_types.h>

#include "helpers.h"
#include "mock_server.h"
#include "model_library_interface.h"

using ::testing::_;
using ::testing::Invoke;
using ::testing::WithArgs;
using namespace research_interface::robot;

struct MockModel : public ModelLibraryInterface {
  MOCK_METHOD1(Ji_J_J1, void(double*));
  MOCK_METHOD2(Ji_J_J2, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J3, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J4, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J5, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J6, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J7, void(const double*, double*));
  MOCK_METHOD2(Ji_J_J8, void(const double*, double*));
  MOCK_METHOD3(Ji_J_J9, void(const double*, const double*, double*));

  MOCK_METHOD5(M_NE, void(const double*, const double*, double, const double*, double*));

  MOCK_METHOD1(O_J_J1, void(double*));
  MOCK_METHOD2(O_J_J2, void(const double*, double*));
  MOCK_METHOD2(O_J_J3, void(const double*, double*));
  MOCK_METHOD2(O_J_J4, void(const double*, double*));
  MOCK_METHOD2(O_J_J5, void(const double*, double*));
  MOCK_METHOD2(O_J_J6, void(const double*, double*));
  MOCK_METHOD2(O_J_J7, void(const double*, double*));
  MOCK_METHOD2(O_J_J8, void(const double*, double*));
  MOCK_METHOD3(O_J_J9, void(const double*, const double*, double*));

  MOCK_METHOD2(O_T_J1, void(const double*, double*));
  MOCK_METHOD2(O_T_J2, void(const double*, double*));
  MOCK_METHOD2(O_T_J3, void(const double*, double*));
  MOCK_METHOD2(O_T_J4, void(const double*, double*));
  MOCK_METHOD2(O_T_J5, void(const double*, double*));
  MOCK_METHOD2(O_T_J6, void(const double*, double*));
  MOCK_METHOD2(O_T_J7, void(const double*, double*));
  MOCK_METHOD2(O_T_J8, void(const double*, double*));
  MOCK_METHOD3(O_T_J9, void(const double*, const double*, double*));

  MOCK_METHOD6(c_NE,
               void(const double*, const double*, const double*, double, const double*, double*));
  MOCK_METHOD5(g_NE, void(const double*, const double*, double, const double*, double*));
};

struct Model : public ::testing::Test {
  Model() {
    using namespace std::string_literals;

    std::ifstream model_library_stream(
        FRANKA_TEST_BINARY_DIR + "/libfcimodels.so"s,
        std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
    std::vector<char> buffer;
    buffer.resize(model_library_stream.tellg());
    model_library_stream.seekg(0, std::ios::beg);
    if (!model_library_stream.read(buffer.data(), buffer.size())) {
      throw std::runtime_error("Model test: Cannot load mock libfcimodels.so");
    }

    server
        .generic([=](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
          CommandHeader header;
          server.receiveRequest<LoadModelLibrary>(tcp_socket, &header);
          server.sendResponse<LoadModelLibrary>(
              tcp_socket,
              CommandHeader(Command::kLoadModelLibrary, header.command_id,
                            sizeof(CommandMessage<LoadModelLibrary::Response>) + buffer.size()),
              LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess));
          tcp_socket.sendBytes(buffer.data(), buffer.size());
        })
        .spinOnce();

    model_library_interface = nullptr;
  }

  RobotMockServer server{};
  franka::Robot robot{"127.0.0.1"};
};

TEST(InvalidModel, ThrowsIfNoModelReceived) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1");

  server
      .waitForCommand<LoadModelLibrary>([&](const LoadModelLibrary::Request&) {
        return LoadModelLibrary::Response(LoadModelLibrary::Status::kError);
      })
      .spinOnce();

  EXPECT_THROW(robot.loadModel(), franka::ModelException);
}

TEST(InvalidModel, ThrowsIfInvalidModelReceived) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1");

  std::array<char, 10> buffer{};
  server
      .generic([&](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
        CommandHeader header;
        server.receiveRequest<LoadModelLibrary>(tcp_socket, &header);
        server.sendResponse<LoadModelLibrary>(
            tcp_socket,
            CommandHeader(Command::kLoadModelLibrary, header.command_id,
                          sizeof(CommandMessage<LoadModelLibrary::Response>) + buffer.size()),
            LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess));
        tcp_socket.sendBytes(buffer.data(), buffer.size());
      })
      .spinOnce();

  EXPECT_THROW(robot.loadModel(), franka::ModelException);
}

TEST_F(Model, CanCreateModel) {
  EXPECT_NO_THROW(robot.loadModel());
}

TEST_F(Model, CanGetMassMatrix) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  MockModel mock;
  EXPECT_CALL(mock, M_NE(robot_state.q.data(), robot_state.I_total.data(), robot_state.m_total,
                         robot_state.F_x_Ctotal.data(), _))
      .WillOnce(WithArgs<4>(Invoke([=](double* output) {
        for (size_t i = 0; i < 49; i++) {
          output[i] = i;
        }
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  auto matrix = model.mass(robot_state);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetCoriolisVector) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 7> expected_vector{12, 13, 14, 15, 16, 17, 18};

  MockModel mock;
  EXPECT_CALL(mock, c_NE(robot_state.q.data(), robot_state.dq.data(), robot_state.I_total.data(),
                         robot_state.m_total, robot_state.F_x_Ctotal.data(), _))
      .WillOnce(WithArgs<5>(Invoke([=](double* output) {
        std::copy(expected_vector.cbegin(), expected_vector.cend(), output);
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  auto vector = model.coriolis(robot_state);
  EXPECT_EQ(expected_vector, vector);
}

TEST_F(Model, CanGetGravity) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 3> gravity_earth{4, 5, 6};

  MockModel mock;
  EXPECT_CALL(mock, g_NE(robot_state.q.data(), gravity_earth.data(), robot_state.m_total,
                         robot_state.F_x_Ctotal.data(), _))
      .WillOnce(WithArgs<4>(Invoke([=](double* output) {
        for (size_t i = 0; i < 7; i++) {
          output[i] = i;
        }
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  auto matrix = model.gravity(robot_state, gravity_earth);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetJointPoses) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  std::array<double, 16> expected_pose{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

  MockModel mock;
  EXPECT_CALL(mock, O_T_J1(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J2(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J3(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J4(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J5(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J6(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J7(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J8(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J9(robot_state.q.data(), _, _))
      .WillOnce(WithArgs<1, 2>(Invoke([=](const double* input, double* output) {
        std::array<double, 16> expected;
        Eigen::Map<Eigen::Matrix4d>(expected.data(), 4, 4) =
            (Eigen::Matrix4d(robot_state.F_T_EE.data()) *
             Eigen::Matrix4d(robot_state.EE_T_K.data()));
        std::array<double, 16> input_array;
        std::copy(&input[0], &input[16], input_array.data());
        EXPECT_EQ(expected, input_array);
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J9(robot_state.q.data(), robot_state.F_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto pose = model.pose(joint, robot_state);
    EXPECT_EQ(expected_pose, pose);
  }
}

TEST_F(Model, CanGetBodyJacobian) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  std::array<double, 42> expected_jacobian;
  for (unsigned int i = 0; i < expected_jacobian.size(); i++) {
    expected_jacobian[i] = i;
  }

  MockModel mock;
  EXPECT_CALL(mock, Ji_J_J1(_)).WillOnce(WithArgs<0>(Invoke([=](double* output) {
    std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
  })));
  EXPECT_CALL(mock, Ji_J_J2(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J3(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J4(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J5(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J6(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J7(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J8(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J9(robot_state.q.data(), _, _))
      .WillOnce(WithArgs<1, 2>(Invoke([=](const double* input, double* output) {
        std::array<double, 16> expected;
        Eigen::Map<Eigen::Matrix4d>(expected.data(), 4, 4) =
            (Eigen::Matrix4d(robot_state.F_T_EE.data()) *
             Eigen::Matrix4d(robot_state.EE_T_K.data()));
        std::array<double, 16> input_array;
        std::copy(&input[0], &input[16], input_array.data());
        EXPECT_EQ(expected, input_array);
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, Ji_J_J9(robot_state.q.data(), robot_state.F_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto jacobian = model.bodyJacobian(joint, robot_state);
    EXPECT_EQ(expected_jacobian, jacobian);
  }
}

TEST_F(Model, CanGetZeroJacobian) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  std::array<double, 42> expected_jacobian;
  for (unsigned int i = 0; i < expected_jacobian.size(); i++) {
    expected_jacobian[i] = i;
  }

  MockModel mock;
  EXPECT_CALL(mock, O_J_J1(_)).WillOnce(WithArgs<0>(Invoke([=](double* output) {
    std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
  })));
  EXPECT_CALL(mock, O_J_J2(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J3(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J4(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J5(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J6(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J7(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J8(robot_state.q.data(), _))
      .WillOnce(WithArgs<1>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J9(robot_state.q.data(), _, _))
      .WillOnce(WithArgs<1, 2>(Invoke([=](const double* input, double* output) {
        std::array<double, 16> expected;
        Eigen::Map<Eigen::Matrix4d>(expected.data(), 4, 4) =
            Eigen::Matrix4d(Eigen::Matrix4d(robot_state.F_T_EE.data()) *
                            Eigen::Matrix4d(robot_state.EE_T_K.data()));
        std::array<double, 16> input_array;
        std::copy(&input[0], &input[16], input_array.data());
        EXPECT_EQ(expected, input_array);
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));
  EXPECT_CALL(mock, O_J_J9(robot_state.q.data(), robot_state.F_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_jacobian.cbegin(), expected_jacobian.cend(), output);
      })));

  model_library_interface = &mock;

  franka::Model model(robot.loadModel());
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kStiffness; joint++) {
    auto jacobian = model.zeroJacobian(joint, robot_state);
    EXPECT_EQ(expected_jacobian, jacobian);
  }
}

TEST(Frame, CanIncrement) {
  franka::Frame frame = franka::Frame::kJoint3;
  EXPECT_EQ(franka::Frame::kJoint3, frame++);
  EXPECT_EQ(franka::Frame::kJoint4, frame);
}
