#pragma once

#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <research_interface/gripper/types.h>

#include "network.h"

namespace franka {

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept;

class Gripper::Impl {
 public:
  static constexpr std::chrono::seconds kDefaultTimeout{5};

  explicit Impl(std::unique_ptr<Network> network);

  GripperState readOnce();

  ServerVersion serverVersion() const noexcept;

  template <typename T, typename... TArgs>
  void executeCommand(TArgs...);  // NOLINT (readability-named-parameter)

 private:
  template <typename T>
  void handleCommandResponse(const typename T::Response& response);

  research_interface::gripper::GripperState receiveGripperState();

  std::unique_ptr<Network> network_;

  uint16_t ri_version_;

  research_interface::gripper::GripperState gripper_state_{};
};

template <typename T>
void Gripper::Impl::handleCommandResponse(const typename T::Response& response) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  switch (response.status) {
    case T::Status::kSuccess:
      break;
    case T::Status::kFail:
      throw CommandException("libfranka gripper: command failed!");
    case T::Status::kUnsuccessful:
      throw CommandException("libfranka gripper: command unsuccessful!");
    default:
      throw ProtocolException("libfranka gripper: Unexpected response while handling command!");
  }
}

template <typename T, typename... TArgs>
void Gripper::Impl::executeCommand(TArgs... args) {
  typename T::Request request(std::forward<TArgs>(args)...);
  network_->tcpSendRequest<T>(request);

  typename T::Response response = network_->tcpBlockingReceiveResponse<T>();

  handleCommandResponse<T>(response);
}

}  // namespace franka
