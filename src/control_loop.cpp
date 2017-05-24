#include "control_loop.h"

#include <cstring>

#ifdef _WIN32
#include <Windows.h>
#else
#include <pthread.h>
#endif

#include <franka/exception.h>

// `using std::string_literals::operator""s` produces a GCC warning that cannot
// be disabled, so we have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT (google-build-using-namespace)

namespace franka {

ControlLoop::ControlLoop(RobotControl& robot, ControlCallback control_callback)
    : robot_(robot), control_callback_(std::move(control_callback)) {
  if (control_callback_) {
    setCurrentThreadToRealtime(robot_.realtimeConfig());
    robot_.startController();
  }
}

ControlLoop::~ControlLoop() {
  if (control_callback_) {
    robot_.stopController();
  }
}

void ControlLoop::operator()() {
  while (spinOnce()) {
    robot_.update();
  }
}

bool ControlLoop::spinOnce() {
  if (control_callback_) {
    Torques control_output = control_callback_(robot_.robotState());
    if (control_output.stop()) {
      return false;
    }
    research_interface::ControllerCommand command{};
    command.tau_J_d = control_output.tau_J;
    robot_.controllerCommand(command);
  }

  return true;
}

void ControlLoop::setCurrentThreadToRealtime(RealtimeConfig config) {
#ifdef _WIN32
  // TODO: test on WINDOWS
  auto get_last_windows_error = []() -> std::string {
    DWORD error_id = GetLastError();
    LPSTR buffer = nullptr;
    size_t size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), static_cast<LPSTR>(&buffer),
        0, nullptr);
    return std::string(buffer, size);
  };

  if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)) {
    if (config == RealtimeConfig::kEnforce) {
      throw RealTimeException("libfranka: unable to set realtime priority for the process: "s +
                              get_last_windows_error());
    }
    return;
  }
  if (!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL)) {
    if (config == RealtimeConfig::kEnforce) {
      throw RealTimeException("libfranka: unable to set realtime priority for the thread: "s +
                              get_last_windows_error());
    }
  }
#else
  int policy = SCHED_FIFO;
  struct sched_param thread_param;
  constexpr int kThreadPriority = 20;
  thread_param.sched_priority = kThreadPriority;
  if (pthread_setschedparam(pthread_self(), policy, &thread_param) != 0) {
    if (config == RealtimeConfig::kEnforce) {
      throw RealtimeException("libfranka: unable to set realtime scheduling: "s + strerror(errno));
    }
  }
#endif
}

}  // namespace franka
