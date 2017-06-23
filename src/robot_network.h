#include <chrono>
#include <cstring>
#include <functional>
#include <unordered_set>

#include <research_interface/robot/rbk_types.h>
#include <research_interface/robot/service_types.h>

#include "network.h"

namespace franka {

class RobotNetwork : public Network {
 public:
  explicit RobotNetwork(const std::string& franka_address,
                        uint16_t franka_port,
                        std::chrono::milliseconds timeout);

  ~RobotNetwork() override = default;

  research_interface::robot::RobotState udpReadRobotState();

  void udpSendRobotCommand(const research_interface::robot::RobotCommand& command);

  bool tcpReadResponse(research_interface::robot::Function* function);

 private:
  const char* getName() override { return "robot"; };
};

}  // namespace franka
