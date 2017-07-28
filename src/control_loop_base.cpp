#include "control_loop_base.h"

#include <pthread.h>

#include <cstring>
#include <fstream>

#include <franka/exception.h>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

constexpr research_interface::robot::Move::Deviation ControlLoopBase::kDefaultDeviation;

ControlLoopBase::ControlLoopBase(RobotControl& robot, ControlCallback control_callback)
    : robot_(robot), control_callback_(std::move(control_callback)) {
  bool throw_on_error = robot_.realtimeConfig() == RealtimeConfig::kEnforce;
  if (throw_on_error && !hasRealtimeKernel()) {
    throw RealtimeException("libfranka: Running kernel does not have realtime capabilities.");
  }
  setCurrentThreadToRealtime(throw_on_error);
}

bool ControlLoopBase::spinOnce(const RobotState& robot_state,
                               franka::Duration time_step,
                               research_interface::robot::ControllerCommand* command) {
  Torques control_output = control_callback_(robot_state, time_step);
  if (control_output.stop()) {
    return false;
  }
  command->tau_J_d = control_output.tau_J;
  return true;
}

ControlLoopBase::~ControlLoopBase() noexcept {
  try {
    robot_.stopMotion();
  } catch (...) {
  }
}

void setCurrentThreadToRealtime(bool throw_on_error) {
  constexpr int kThreadPriority = 20;
  sched_param thread_param{};
  thread_param.sched_priority = kThreadPriority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    if (throw_on_error) {
      throw RealtimeException("libfranka: unable to set realtime scheduling: "s + strerror(errno));
    }
  }
}

bool hasRealtimeKernel() {
  std::ifstream realtime("/sys/kernel/realtime", std::ios_base::in);
  bool is_realtime;
  realtime >> is_realtime;
  return is_realtime;
}

}  // namespace franka
