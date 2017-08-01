#include <franka/gripper.h>

#include <sstream>

#include <franka/exception.h>
#include <research_interface/gripper/types.h>

#include "lock.h"
#include "network.h"

namespace franka {

namespace {

template <typename T, typename... TArgs>
bool executeCommand(Network& network, uint64_t command_id, TArgs&&... args) {
  using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

  network.tcpSendRequest<T>(command_id, std::forward<TArgs>(args)...);
  typename T::Response response = network.tcpBlockingReceiveResponse<T>(command_id);

  switch (response.status) {
    case T::Status::kSuccess:
      return true;
    case T::Status::kFail:
      throw CommandException("libfranka gripper: Command failed!");
    case T::Status::kUnsuccessful:
      return false;
    default:
      throw ProtocolException("libfranka gripper: Unexpected response while handling command!");
  }
}

GripperState convertGripperState(
    const research_interface::gripper::GripperState& gripper_state) noexcept {
  GripperState converted;
  converted.width = gripper_state.width;
  converted.max_width = gripper_state.max_width;
  converted.is_grasped = gripper_state.is_grasped;
  converted.temperature = gripper_state.temperature;
  converted.time = Duration(gripper_state.message_id);
  return converted;
}

}  // anonymous namespace

Gripper::Gripper(const std::string& franka_address)
    : network_{std::make_unique<Network>(franka_address,
                                         research_interface::gripper::kCommandPort)},
      lock_{new Lock} {
  connect<research_interface::gripper::Connect, research_interface::gripper::kVersion>(
      *network_, lock_->command_id++, &ri_version_);
}

Gripper::~Gripper() noexcept = default;

Gripper::ServerVersion Gripper::serverVersion() const noexcept {
  return ri_version_;
}

bool Gripper::homing() {
  return executeCommand<research_interface::gripper::Homing>(*network_, lock_->command_id++);
}

bool Gripper::grasp(double width, double speed, double force) {
  return executeCommand<research_interface::gripper::Grasp>(*network_, lock_->command_id++, width,
                                                            speed, force);
}

bool Gripper::move(double width, double speed) {
  return executeCommand<research_interface::gripper::Move>(*network_, lock_->command_id++, width,
                                                           speed);
}

bool Gripper::stop() {
  return executeCommand<research_interface::gripper::Stop>(*network_, lock_->command_id++);
}

GripperState Gripper::readOnce() const {
  std::unique_lock<std::mutex> l(lock_->mutex, std::try_to_lock);
  if (!l.owns_lock()) {
    throw InvalidOperationException(
        "libfranka gripper: Cannot perform this operation while another controller or motion "
        "generator is running.");
  }

  // Delete old data from the UDP buffer.
  while (network_->udpAvailableData() > 0) {
    network_->udpRead<research_interface::gripper::GripperState>();
  }

  return convertGripperState(network_->udpRead<research_interface::gripper::GripperState>());
}

}  // namespace franka
