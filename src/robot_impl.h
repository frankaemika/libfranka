#pragma once

#include <chrono>

#include <franka/robot.h>
#include <research_interface/rbk_types.h>
#include <research_interface/service_traits.h>
#include <research_interface/service_types.h>

#include "complete_robot_state.h"
#include "network.h"
#include "robot_control.h"

namespace franka {

class Robot::Impl : public RobotControl {
 public:
  static constexpr std::chrono::seconds kDefaultTimeout{5};

  explicit Impl(const std::string& franka_address,
                uint16_t franka_port = research_interface::kCommandPort,
                std::chrono::milliseconds timeout = kDefaultTimeout,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);

  bool update() override;

  void controllerCommand(
      const research_interface::ControllerCommand& controller_command) noexcept override;
  void motionGeneratorCommand(
      const research_interface::MotionGeneratorCommand& motion_generator_command) noexcept override;
  const RobotState& robotState() const noexcept override;
  ServerVersion serverVersion() const noexcept;
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  void startMotionGenerator(
      research_interface::Move::MotionGeneratorMode mode) override;
  void stopMotionGenerator() override;

  void startMotion(research_interface::Move::ControllerMode controller_mode, research_interface::Move::MotionGeneratorMode motion_generator_mode, const research_interface::Move::Deviation& maximum_path_deviation, const research_interface::Move::Deviation& maximum_goal_pose_deviation) override;
  void stopMotion() override;

  template <typename T, typename... TArgs>
  void executeCommand(TArgs...);  // NOLINT (readability-named-parameter)

 private:
  template <typename T>
  void handleCommandResponse(const typename T::Response& response);

  Network network_;

  const RealtimeConfig realtime_config_;
  uint16_t ri_version_;

  research_interface::RobotCommand robot_command_{};
  CompleteRobotState robot_state_{};
};

template <typename T>
void Robot::Impl::handleCommandResponse(const typename T::Response& response) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case T::Status::kSuccess:
      break;
    case T::Status::kAborted:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<T>::kName +
                             " command aborted!");
    case T::Status::kRejected:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<T>::kName +
                             " command rejected!");
    case T::Status::kPreempted:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<T>::kName +
                             " command preempted!");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s + research_interface::CommandTraits<T>::kName + " command!");
  }
}

template <>
inline void Robot::Impl::handleCommandResponse<research_interface::Move>(const research_interface::Move::Response& response) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case research_interface::Move::Status::kSuccess:
      if (!motionGeneratorRunning()) {
        throw ProtocolException("libfranka: "s + research_interface::CommandTraits<research_interface::Move>::kName + " received unexpected motion finished message.");
      }
      break;
    case research_interface::Move::Status::kMotionStarted:
      if (motionGeneratorRunning()) {
        throw ProtocolException("libfranka: "s + research_interface::CommandTraits<research_interface::Move>::kName + " received unexpected motion started message.");
      }
      break;
    case research_interface::Move::Status::kAborted:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<research_interface::Move>::kName +
                             " command aborted!");
    case research_interface::Move::Status::kRejected:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<research_interface::Move>::kName +
                             " command rejected!");
    case research_interface::Move::Status::kPreempted:
      throw CommandException("libfranka: "s + research_interface::CommandTraits<research_interface::Move>::kName +
                             " command preempted!");
    default:
      throw ProtocolException("libfranka: Unexpected response while handling "s + research_interface::CommandTraits<research_interface::Move>::kName + " command!");
  }
}

template <typename T, typename... TArgs>
void Robot::Impl::executeCommand(TArgs... args) {
  typename T::Request request(std::forward<TArgs>(args)...);
  network_.tcpSendRequest(request);

  typename T::Response response = network_.tcpBlockingReceiveResponse<T>();

  handleCommandResponse<T>(response);
}

template <>
inline void
Robot::Impl::executeCommand<research_interface::GetCartesianLimit, int32_t, VirtualWallCuboid*>(
    int32_t id,
    VirtualWallCuboid* virtual_wall_cuboid) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  research_interface::GetCartesianLimit::Request request(id);
  network_.tcpSendRequest(request);

  research_interface::GetCartesianLimit::Response response =
      network_.tcpBlockingReceiveResponse<research_interface::GetCartesianLimit>();
  virtual_wall_cuboid->p_frame = response.object_frame;
  virtual_wall_cuboid->p_max = response.object_p_max;
  virtual_wall_cuboid->p_min = response.object_p_min;
  virtual_wall_cuboid->active = response.object_activation;
  virtual_wall_cuboid->id = id;

  handleCommandResponse<research_interface::GetCartesianLimit>(response);
}

}  // namespace franka
