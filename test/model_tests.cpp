#include <fstream>
#include <memory>

#include <gmock/gmock.h>

#include <franka/model.h>
#include <franka/robot.h>
#include <research_interface/robot/service_types.h>

#include "helpers.h"
#include "mock_server.h"
#include "model_library_interface.h"

using ::testing::_;
using ::testing::WithArgs;
using ::testing::Invoke;
using namespace research_interface::robot;

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
  MOCK_METHOD5(M_NE, void(const double*, const double*, double, const double*, double*));
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
    buffer.resize(model_library_stream.tellg());
    model_library_stream.seekg(0, std::ios::beg);
    if (!model_library_stream.read(buffer.data(), buffer.size())) {
      throw std::runtime_error("Model test: Cannot load mock libfcimodels.so");
    }

    server
        .generic([&](MockServer::Socket& tcp_socket, MockServer::Socket&) {
          server.handleCommand<LoadModelLibrary>(tcp_socket, [&](auto) {
            return LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess, buffer.size());
          });
          tcp_socket.sendBytes(buffer.data(), buffer.size());
        })
        .spinOnce();

    model_library_interface = nullptr;
  }

  MockServer server{};
  franka::Robot robot{"127.0.0.1"};

 private:
  std::vector<char> buffer;
};

TEST(InvalidModel, ThrowsIfNoModelReceived) {
  MockServer server;
  franka::Robot robot("127.0.0.1");

  server
      .waitForCommand<LoadModelLibrary>(
          [&](auto) { return LoadModelLibrary::Response(LoadModelLibrary::Status::kError, 0); })
      .spinOnce();

  EXPECT_THROW(std::unique_ptr<franka::Model>(robot.loadModel()), franka::ModelException);
}

TEST(InvalidModel, ThrowsIfInvalidModelReceived) {
  MockServer server;
  franka::Robot robot("127.0.0.1");

  std::array<char, 10> buffer{};
  server
      .generic([&](MockServer::Socket& tcp_socket, MockServer::Socket&) {
        server.handleCommand<LoadModelLibrary>(tcp_socket, [&](auto) {
          return LoadModelLibrary::Response(LoadModelLibrary::Status::kSuccess, buffer.size());
        });
        tcp_socket.sendBytes(buffer.data(), buffer.size());
      })
      .spinOnce();

  EXPECT_THROW(std::unique_ptr<franka::Model> model(robot.loadModel()), franka::ModelException);
}

TEST_F(Model, CanCreateModel) {
  EXPECT_NO_THROW(std::unique_ptr<franka::Model> model(robot.loadModel()));
}

TEST_F(Model, CanGetMassMatrix) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  std::array<double, 9> load_inertia{0, 1, 2, 3, 4, 5, 6, 7, 8};
  double load_mass = 0.75;
  std::array<double, 3> F_x_Cload{9, 10, 11};

  MockModel mock;
  EXPECT_CALL(mock, M_NE(robot_state.q.data(), load_inertia.data(), load_mass, F_x_Cload.data(), _))
      .WillOnce(WithArgs<4>(Invoke([=](double* output) {
        for (size_t i = 0; i < 49; i++) {
          output[i] = i;
        }
      })));

  model_library_interface = &mock;

  std::unique_ptr<franka::Model> model(robot.loadModel());
  auto matrix = model->mass(robot_state, load_inertia, load_mass, F_x_Cload);
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
  EXPECT_CALL(mock, c_NE(robot_state.q.data(), robot_state.dq.data(), load_inertia.data(),
                         load_mass, F_x_Cload.data(), _))
      .WillOnce(WithArgs<5>(Invoke([=](double* output) {
        std::copy(expected_vector.cbegin(), expected_vector.cend(), output);
      })));

  model_library_interface = &mock;

  std::unique_ptr<franka::Model> model(robot.loadModel());
  auto vector = model->coriolis(robot_state, load_inertia, load_mass, F_x_Cload);
  EXPECT_EQ(expected_vector, vector);
}

TEST_F(Model, CanGetGravity) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);
  double load_mass = 0.75;
  std::array<double, 3> F_x_Cload{1, 2, 3};
  std::array<double, 3> gravity_earth{4, 5, 6};

  MockModel mock;
  EXPECT_CALL(mock,
              g_NE(robot_state.q.data(), gravity_earth.data(), load_mass, F_x_Cload.data(), _))
      .WillOnce(WithArgs<4>(Invoke([=](double* output) {
        for (size_t i = 0; i < 7; i++) {
          output[i] = i;
        }
      })));

  model_library_interface = &mock;

  std::unique_ptr<franka::Model> model(robot.loadModel());
  auto matrix = model->gravity(robot_state, load_mass, F_x_Cload, gravity_earth);
  for (size_t i = 0; i < matrix.size(); i++) {
    EXPECT_EQ(i, matrix[i]);
  }
}

TEST_F(Model, CanGetJointPoses) {
  franka::RobotState robot_state;
  randomRobotState(robot_state);

  std::array<double, 16> expected_pose{{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};

  MockModel mock;
  EXPECT_CALL(mock, O_T_J1(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J2(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J3(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J4(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J5(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J6(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J7(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J8(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));
  EXPECT_CALL(mock, O_T_J9(robot_state.q.data(), robot_state.O_T_EE.data(), _))
      .WillOnce(WithArgs<2>(Invoke([=](double* output) {
        std::copy(expected_pose.cbegin(), expected_pose.cend(), output);
      })));

  model_library_interface = &mock;

  std::unique_ptr<franka::Model> model(robot.loadModel());
  for (franka::Frame joint = franka::Frame::kJoint1; joint <= franka::Frame::kEndEffector;
       joint = static_cast<franka::Frame>(joint + 1)) {
    auto pose = model->jointPose(joint, robot_state);
    EXPECT_EQ(expected_pose, pose);
  }
}
