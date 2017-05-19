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

  void controllerCommand(const research_interface::ControllerCommand&
                             controller_command) noexcept override;
  void motionGeneratorCommand(const research_interface::MotionGeneratorCommand&
                                  motion_generator_command) noexcept override;
  const RobotState& robotState() const noexcept override;
  ServerVersion serverVersion() const noexcept;
  bool motionGeneratorRunning() const noexcept;
  bool controllerRunning() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  void startMotionGenerator(
      research_interface::StartMotionGenerator::MotionGeneratorMode mode)
      override;
  void stopMotionGenerator() override;

  template <typename T, typename... TArgs>
  void executeCommand(TArgs...);  // NOLINT (readability-named-parameter)

  template <research_interface::StartMotionGenerator::MotionGeneratorMode,
            typename T>
  void control(
      std::function<T(const RobotState&)> control_callback,
      std::function<void(const T&, research_interface::MotionGeneratorCommand*)>
          conversion_callback);

  template <typename T>
  void control(
      std::function<T(const RobotState&)> control_callback,
      std::function<void(const T&, research_interface::ControllerCommand*)>
          conversion_callback);

 private:
  template <typename T>
  bool handleCommandResponse(const typename T::Response& response);

  Network network_;

  const RealtimeConfig realtime_config_;
  uint16_t ri_version_;
  bool motion_generator_running_;
  bool controller_running_;

  research_interface::RobotCommand robot_command_;
  CompleteRobotState robot_state_;

  // Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=60970
  // taken from http://stackoverflow.com/a/24847480/249642
  struct EnumClassHash {
    template <typename T>
    size_t operator()(T t) const {
      return static_cast<size_t>(t);
    }
  };
  std::unordered_multiset<research_interface::Function, EnumClassHash>
      expected_responses_;
};

template <typename T, typename... TArgs>
void Robot::Impl::executeCommand(TArgs... args) {
  typename T::Request request(std::forward<TArgs>(args)...);
  network_.tcpSendRequest(request);

  typename T::Response response = network_.tcpBlockingReceiveResponse<T>();

  switch (response.status) {
    case T::Status::kSuccess:
      break;
    case T::Status::kAborted:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<T>::kName) +
          " command aborted!");
    case T::Status::kRejected:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<T>::kName) +
          " command rejected!");
    case T::Status::kPreempted:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<T>::kName) +
          " command preempted!");
  }
}

template <>
inline void Robot::Impl::executeCommand<research_interface::GetCartesianLimit,
                                        int32_t,
                                        VirtualWallCuboid*>(
    int32_t id,
    VirtualWallCuboid* virtual_wall_cuboid) {
  research_interface::GetCartesianLimit::Request request(id);
  network_.tcpSendRequest(request);

  research_interface::GetCartesianLimit::Response response =
      network_
          .tcpBlockingReceiveResponse<research_interface::GetCartesianLimit>();
  virtual_wall_cuboid->p_frame = response.object_frame;
  virtual_wall_cuboid->p_max = response.object_p_max;
  virtual_wall_cuboid->p_min = response.object_p_min;
  virtual_wall_cuboid->active = response.object_activation;
  virtual_wall_cuboid->id = id;

  switch (response.status) {
    case research_interface::GetCartesianLimit::Status::kSuccess:
      break;
    case research_interface::GetCartesianLimit::Status::kAborted:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<
                      research_interface::GetCartesianLimit>::kName) +
          " command aborted!");
    case research_interface::GetCartesianLimit::Status::kRejected:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<
                      research_interface::GetCartesianLimit>::kName) +
          " command rejected!");
    case research_interface::GetCartesianLimit::Status::kPreempted:
      throw CommandException(
          "libfranka: " +
          std::string(research_interface::CommandTraits<
                      research_interface::GetCartesianLimit>::kName) +
          " command preempted!");
  }
}

}  // namespace franka
