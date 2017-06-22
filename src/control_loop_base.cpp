#include "control_loop_base.h"

#include <cstring>

#include <pthread.h>

#include <franka/exception.h>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

ControlLoopBase::ControlLoopBase(RobotControl& robot, ControlCallback control_callback)
    : robot_(robot), control_callback_(std::move(control_callback)) {
  setCurrentThreadToRealtime(robot_.realtimeConfig());
}

bool ControlLoopBase::spinOnce(const RobotState& robot_state,
                               research_interface::robot::ControllerCommand* command) {
  if (control_callback_) {
    Torques control_output = control_callback_(robot_state);
    if (control_output.stop()) {
      return false;
    }
    command->tau_J_d = control_output.tau_J;
  }

  return true;
}

void setCurrentThreadToRealtime(RealtimeConfig config) {
  int policy = SCHED_FIFO;
  struct sched_param thread_param;
  constexpr int kThreadPriority = 20;
  thread_param.sched_priority = kThreadPriority;
  if (pthread_setschedparam(pthread_self(), policy, &thread_param) != 0) {
    if (config == RealtimeConfig::kEnforce) {
      throw RealtimeException("libfranka: unable to set realtime scheduling: "s + strerror(errno));
    }
  }
}

}  // namespace franka
