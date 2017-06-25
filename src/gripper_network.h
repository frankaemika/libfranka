#include <chrono>
#include <cstring>
#include <functional>
#include <unordered_set>

#include <research_interface/gripper/types.h>

#include "network.h"

namespace franka {

class GripperNetwork : public Network {
 public:
  explicit GripperNetwork(const std::string& franka_address,
                          uint16_t franka_port,
                          std::chrono::milliseconds timeout);

  ~GripperNetwork() override = default;

  research_interface::gripper::GripperState udpReadGripperState();

  bool tcpReadResponse(research_interface::gripper::Function* function);

 private:
  const char* getName() override { return "gripper"; };
};

}  // namespace franka
