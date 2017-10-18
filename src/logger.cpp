#include "logger.h"

#include <iterator>
#include <sstream>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>
#include <fstream>

using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace {

template <typename T, size_t N>
std::string csvName(const std::array<T, N>&, const std::string& name) {
  std::ostringstream os;
  for (size_t i = 0; i < N - 1; i++) {
    os << name << "[" << i << "], ";
  }
  os << name << "[" << N - 1 << "]";
  return os.str();
}

template <class T, size_t N>
std::string commaSeparated(const std::array<T, N>& array) {
  std::ostringstream os;
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(os, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(os));
  return os.str();
}

std::string csvHeader(const franka::RobotState& robot_state) {
  std::ostringstream os;
  os << "duration, success rate, " << csvName(robot_state.q, "q") << ","
     << csvName(robot_state.q_d, "q_d") << "," << csvName(robot_state.dq, "dq") << ","
     << csvName(robot_state.dq_d, "dq_d") << ","
     << csvName(robot_state.tau_ext_hat_filtered, "tau_ext_hat_filtered");
  return os.str();
}

std::string csvLine(const franka::RobotState& robot_state) {
  std::ostringstream os;
  os << robot_state.time.toMSec() << "," << robot_state.control_command_success_rate << ","
     << commaSeparated(robot_state.q) << "," << commaSeparated(robot_state.q_d) << ","
     << commaSeparated(robot_state.dq) << "," << commaSeparated(robot_state.dq_d) << ","
     << commaSeparated(robot_state.tau_ext_hat_filtered);
  return os.str();
}

std::string csvHeader(const research_interface::robot::RobotCommand& command) {
  std::ostringstream os;
  os << "sent commands,id," << csvName(command.motion.q_d, "q_d") << ","
     << csvName(command.motion.dq_d, "dq_d") << ","
     << csvName(command.motion.O_T_EE_d, "O_T_EE_d") << ","
     << csvName(command.motion.O_dP_EE_d, "O_dP_EE_d") << ","
     << csvName(command.control.tau_J_d, "tau_J_d");
  return os.str();
}

std::string csvLine(const research_interface::robot::RobotCommand& command) {
  std::ostringstream os;
  os << command.message_id << "," << commaSeparated(command.motion.q_d) << ","
     << commaSeparated(command.motion.dq_d) << "," << commaSeparated(command.motion.O_T_EE_d) << ","
     << commaSeparated(command.motion.O_dP_EE_d) << "," << commaSeparated(command.control.tau_J_d);
  return os.str();
}

}  // anonymous namespace

namespace franka {

Logger::Logger(size_t log_size) : log_size_(log_size) {
}

void Logger::log(RobotState state, research_interface::robot::RobotCommand command) {
  if (command_log_.size() >= log_size_) {
    command_log_.pop();
  }
  command_log_.push(command);

  if (state_log_.size() >= log_size_) {
    state_log_.pop();
  }
  state_log_.push(state);
}

std::string Logger::writeToFile() try {
  if (state_log_.empty()) {
    return "";
  }

  Poco::Path temp_dir_path(Poco::Path::temp());
  temp_dir_path.pushDirectory("libfranka-logs");

  Poco::File temp_dir(temp_dir_path);
  temp_dir.createDirectories();

  std::string now_string =
      Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
  Poco::File log_file(Poco::Path(temp_dir_path, "log-"s + now_string + ".csv"));
  if (!log_file.createFile()) {
    return "";
  }

  std::ofstream log_stream(log_file.path().c_str());

  log_stream << csvHeader(state_log_.front()) << ",";
  log_stream << csvHeader(research_interface::robot::RobotCommand{}) << std::endl;
  while (!state_log_.empty()) {
    log_stream << csvLine(state_log_.front());
    if (!command_log_.empty()) {
      log_stream << ",," << csvLine(command_log_.front());
      command_log_.pop();
    }
    log_stream << std::endl;
    state_log_.pop();
  }
  return log_file.path();
} catch (...) {
  return "";
}

}  // namespace franka