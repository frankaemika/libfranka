#pragma once

#include <chrono>
#include <unordered_set>

#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/StreamSocket.h>

#include <franka/robot.h>
#include <research_interface/rbk_types.h>
#include <research_interface/types.h>

#include "complete_robot_state.h"
#include "robot_control.h"

namespace franka {

class Robot::Impl : public RobotControl {
 public:
  static constexpr std::chrono::seconds kDefaultTimeout{5};
  static constexpr double kCommandTimeStep{0.001};

  explicit Impl(const std::string& franka_address,
                uint16_t franka_port = research_interface::kCommandPort,
                std::chrono::milliseconds timeout = kDefaultTimeout,
                RealtimeConfig realtime_config = RealtimeConfig::kEnforce);
  ~Impl() noexcept;

  void setRobotState(const research_interface::RobotState& robot_state);
  bool update() override;

  research_interface::ControllerCommand& controllerCommand() noexcept override;
  research_interface::MotionGeneratorCommand& motionGeneratorCommand() noexcept override;
  const RobotState& robotState() const noexcept override;
  ServerVersion serverVersion() const noexcept;
  bool motionGeneratorRunning() const noexcept;
  RealtimeConfig realtimeConfig() const noexcept override;

  void startController() override;
  void stopController() override;

  void startMotionGenerator(
      research_interface::StartMotionGeneratorRequest::Type
          motion_generator_type) override;
  void stopMotionGenerator() override;

  template <research_interface::StartMotionGeneratorRequest::Type, typename T>
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
  bool handleReplies();

  void receiveRobotState(Poco::Net::SocketAddress* server_address);

  template <typename T>
  void handleReply(std::function<void(T)> handle);

  void handleStartMotionGeneratorReply(
      const research_interface::StartMotionGeneratorReply& reply);
  void handleStopMotionGeneratorReply(
      const research_interface::StopMotionGeneratorReply& reply);
  void handleStartControllerReply(
      const research_interface::StartControllerReply& reply);
  void handleStopControllerReply(
      const research_interface::StopControllerReply& reply);

  template <research_interface::Function F, typename T>
  T tcpReceiveObject();

  const RealtimeConfig realtime_config_;

  uint16_t ri_version_;
  bool motion_generator_running_;
  bool controller_running_;
  research_interface::RobotCommand robot_command_;
  CompleteRobotState robot_state_;
  Poco::Net::StreamSocket tcp_socket_;
  Poco::Net::DatagramSocket udp_socket_;

  std::vector<uint8_t> read_buffer_;
  // Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=60970
  // taken from http://stackoverflow.com/a/24847480/249642
  struct EnumClassHash {
    template <typename T>
    size_t operator()(T t) const {
      return static_cast<size_t>(t);
    }
  };
  std::unordered_set<research_interface::Function, EnumClassHash>
      expected_replies_;
};

}  // namespace franka
