#pragma once

#include <chrono>

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
  static constexpr std::chrono::seconds kDefaultTimeout{5};

  explicit Impl(std::unique_ptr<Network> network,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  RobotState update(
      const research_interface::robot::MotionGeneratorCommand* motion_command = nullptr,
      const research_interface::robot::ControllerCommand* control_command = nullptr) override;

  ServerVersion serverVersion() const noexcept;
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  void startMotion(
      research_interface::robot::Move::ControllerMode controller_mode,
      research_interface::robot::Move::MotionGeneratorMode motion_generator_mode,
      const research_interface::robot::Move::Deviation& maximum_path_deviation,
      const research_interface::robot::Move::Deviation& maximum_goal_pose_deviation) override;
  void stopMotion() override;

  template <typename T, typename... TArgs>
  void executeCommand(TArgs...);  // NOLINT (readability-named-parameter)

 private:
  template <typename T>
  void handleCommandResponse(const typename T::Response& response);

  void sendRobotCommand(const research_interface::robot::MotionGeneratorCommand* motion_command,
                        const research_interface::robot::ControllerCommand* control_command);
  research_interface::robot::RobotState receiveRobotState();

  std::unique_ptr<Network> network_;

  const RealtimeConfig realtime_config_;
  uint16_t ri_version_;

  research_interface::robot::MotionGeneratorMode motion_generator_mode_{};
  research_interface::robot::ControllerMode controller_mode_{};
  uint32_t message_id_{};
};

template <typename T>
void Robot::Impl::handleCommandResponse(const typename T::Response& response) {
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
    const research_interface::robot::Move::Response& response) {
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
  typename T::Request request(std::forward<TArgs>(args)...);
  network_->tcpSendRequest(request);

  typename T::Response response = network_->tcpBlockingReceiveResponse<T>();

  handleCommandResponse<T>(response);
}

template <>
inline void Robot::Impl::executeCommand<research_interface::robot::GetCartesianLimit,
                                        int32_t,
                                        VirtualWallCuboid*>(
    int32_t id,
    VirtualWallCuboid* virtual_wall_cuboid) {
  research_interface::robot::GetCartesianLimit::Request request(id);
  network_->tcpSendRequest(request);

  research_interface::robot::GetCartesianLimit::Response response =
      network_->tcpBlockingReceiveResponse<research_interface::robot::GetCartesianLimit>();
  virtual_wall_cuboid->p_frame = response.object_frame;
  virtual_wall_cuboid->p_max = response.object_p_max;
  virtual_wall_cuboid->p_min = response.object_p_min;
  virtual_wall_cuboid->active = response.object_activation;
  virtual_wall_cuboid->id = id;

  handleCommandResponse<research_interface::robot::GetCartesianLimit>(response);
}

}  // namespace franka
