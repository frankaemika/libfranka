#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include <franka/model.h>
#include <franka/robot.h>
#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_traits.h>
#include <research_interface/robot/service_types.h>

#include "network.h"
#include "robot_control.h"

namespace franka {

RobotState convertRobotState(const research_interface::robot::RobotState& robot_state) noexcept;

class Robot::Impl : public RobotControl {
 public:
  explicit Impl(std::unique_ptr<Network> network,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  RobotState update(
      const research_interface::robot::MotionGeneratorCommand* motion_command = nullptr,
      const research_interface::robot::ControllerCommand* control_command = nullptr) override;

  void throwOnMotionError(const RobotState& robot_state, const uint32_t* motion_id) override;

  RobotState readOnce();

  ServerVersion serverVersion() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  uint32_t startMotion(
      research_interface::robot::Move::ControllerMode controller_mode,
      research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::robot::Move::Deviation& maximum_path_deviation,
      const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) override;
  void stopMotion(uint32_t motion_id) override;

  template <typename T, typename... TArgs>
  void executeCommand(TArgs... /* args */);

  template <typename T>
  void executeCommand(const typename T::Request& request);

  Model loadModel();

 protected:
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;

 private:
  RobotState updateUnsafe(
      const research_interface::robot::MotionGeneratorCommand* motion_command = nullptr,
      const research_interface::robot::ControllerCommand* control_command = nullptr);
  void throwOnMotionErrorUnsafe(const RobotState& robot_state, const uint32_t* motion_id);

  template <typename T>
  void handleCommandResponse(const typename T::Response& response) const;

  void sendRobotCommand(const research_interface::robot::MotionGeneratorCommand* motion_command,
                        const research_interface::robot::ControllerCommand* control_command) const;
  research_interface::robot::RobotState receiveRobotState();
  void updateState(const research_interface::robot::RobotState& robot_state);

  std::unique_ptr<Network> network_;

  const RealtimeConfig realtime_config_;
  uint16_t ri_version_;

  std::mutex mutex_;
  std::atomic<research_interface::robot::MotionGeneratorMode> motion_generator_mode_;
  std::atomic<research_interface::robot::ControllerMode> controller_mode_;
  uint64_t message_id_;

  std::atomic<uint32_t> command_id_{0};
};

template <typename T>
void Robot::Impl::handleCommandResponse(const typename T::Response& response) const {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case T::Status::kSuccess:
      break;
    case T::Status::kAborted:
      throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                             " command aborted!");
    case T::Status::kRejected:
      throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                             " command rejected!");
    case T::Status::kPreempted:
      throw CommandException("libfranka: "s + research_interface::robot::CommandTraits<T>::kName +
                             " command preempted!");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s +
                              research_interface::robot::CommandTraits<T>::kName + " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<research_interface::robot::Move>(
    const research_interface::robot::Move::Response& response) const {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case research_interface::robot::Move::Status::kSuccess:
      break;
    case research_interface::robot::Move::Status::kMotionStarted:
      if (motionGeneratorRunning()) {
        throw ProtocolException(
            "libfranka: "s +
            research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
            " received unexpected motion started message.");
      }
      break;
    case research_interface::robot::Move::Status::kAborted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command aborted!");
    case research_interface::robot::Move::Status::kRejected:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command rejected!");
    case research_interface::robot::Move::Status::kPreempted:
      throw CommandException(
          "libfranka: "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command preempted!");
    default:
      throw ProtocolException(
          "libfranka: Unexpected response while handling "s +
          research_interface::robot::CommandTraits<research_interface::robot::Move>::kName +
          " command!");
  }
}

template <typename T, typename... TArgs>
void Robot::Impl::executeCommand(TArgs... args) {
  command_id_++;

  typename T::Request request(command_id_, std::forward<TArgs>(args)...);
  executeCommand<T>(request);
}

template <typename T>
void Robot::Impl::executeCommand(const typename T::Request& request) {
  network_->tcpSendRequest<T>(request);
  typename T::Response response =
      network_->tcpBlockingReceiveResponse<T>(request.header.command_id);

  handleCommandResponse<T>(response);
}

template <>
inline void Robot::Impl::executeCommand<research_interface::robot::GetCartesianLimit,
                                        int32_t,
                                        VirtualWallCuboid*>(
    int32_t id,
    VirtualWallCuboid* virtual_wall_cuboid) {
  command_id_++;

  research_interface::robot::GetCartesianLimit::Request request(command_id_, id);
  network_->tcpSendRequest<research_interface::robot::GetCartesianLimit>(request);

  research_interface::robot::GetCartesianLimit::Response response =
      network_->tcpBlockingReceiveResponse<research_interface::robot::GetCartesianLimit>(
          command_id_);
  virtual_wall_cuboid->p_frame = response.object_frame;
  virtual_wall_cuboid->p_max = response.object_p_max;
  virtual_wall_cuboid->p_min = response.object_p_min;
  virtual_wall_cuboid->active = response.object_activation;
  virtual_wall_cuboid->id = id;

  handleCommandResponse<research_interface::robot::GetCartesianLimit>(response);
}

}  // namespace franka
