#pragma once

#include <chrono>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/StreamSocket.h>

#include <franka/robot.h>
#include <research_interface/constants.h>
#include <research_interface/rbk_types.h>

namespace franka {

class Robot::Impl {
 public:
  static constexpr std::chrono::seconds kDefaultTimeout{5};

  explicit Impl(const std::string& franka_address,
                uint16_t franka_port = research_interface::kCommandPort,
                std::chrono::milliseconds timeout = kDefaultTimeout);
  ~Impl() noexcept;

  void setRobotState(const research_interface::RobotState& robot_state);
  bool waitForRobotState();
  const RobotState& robotState() const noexcept;
  ServerVersion serverVersion() const noexcept;

 protected:
  // Can throw NetworkException and ProtocolException
  template <class T>
  void tcpReceiveObject(T& object);

 private:
  uint16_t ri_version_;
  RobotState robot_state_;

  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;
};

}  // namespace franka
