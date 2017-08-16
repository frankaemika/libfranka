#include <unistd.h>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

/**
 * @example joint_impedance_control.cpp
 * An example showing a joint impedance type control that executes a Cartesian motion in the
 * shape of a circle. The example illustrates how to use the internal inverse kinematics to map
 * a Cartesian trajectory to joint space. The joint space target is tracked by an impedance control
 * that additionally compensates coriolis terms using the libfranka model library. This example also
 * serves to compare commanded vs. measured torques. The results are printed from a separate thread
 * to avoid blocking print functions in the real-time loop.
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 5) {
    std::cerr << "Usage: ./" << argv[0] << " <robot-hostname>"
              << " <radius in [m]>"
              << " <vel_max in [m/s]>"
              << " <print_rate in [Hz]>" << std::endl;
    return -1;
  }

  // set and initialize trajectory parameters
  double radius(std::stod(argv[2]));
  double vel_max(std::stod(argv[3]));
  double acceleration_time(2.0);
  double vel_current(0.0);
  double angle(0.0);

  // set print rate for comparing commanded vs. measured torques
  double print_rate(std::stod(argv[4]));
  if (print_rate < 0.0) {
    std::cerr << "print_rate too small, must be >= 0.0" << std::endl;
    return -1;
  }

  // set gains for the joint impedance control:
  // stiffness:
  std::array<double, 7> k_gains = {{1000.0, 1000.0, 1000.0, 1000.0, 500.0, 300.0, 100.0}};
  // damping:
  std::array<double, 7> d_gains = {{100.0, 100.0, 100.0, 100.0, 50.0, 30.0, 10.0}};

  // initialize data fields for the print thread
  std::array<double, 7> tau_d_last{};
  franka::RobotState robot_state;
  std::array<double, 7> gravity{};
  std::mutex torques_mutex;
  std::atomic_bool running;
  running = true;

  // start print thread
  std::thread print_thread(
      [&tau_d_last, &robot_state, &gravity, &torques_mutex, &running, &print_rate]() {
        if (print_rate > 0.0) {
          while (running) {
            // sleep to achieve the desired print rate
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

            // try to lock data to avoid read write collisions
            if (torques_mutex.try_lock()) {
              std::array<double, 7> tau_error{};
              double error_rms(0.0);
              std::array<double, 7> tau_d_actual{};
              for (size_t i = 0; i < 7; ++i) {
                tau_d_actual[i] = tau_d_last[i] + gravity[i];
                tau_error[i] = tau_d_actual[i] - robot_state.tau_J[i];
                error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
              }
              // print data to console
              std::cout << std::endl
                        << "tau_error[Nm]: " << tau_error << std::endl
                        << "tau_commanded[Nm]: " << tau_d_actual << std::endl
                        << "tau_measured[Nm]: " << robot_state.tau_J << std::endl
                        << "root mean square of tau_error[Nm]: " << error_rms << std::endl
                        << "-----------------------";
              torques_mutex.unlock();
            }
          }
        }
        return;
      });

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    // read the initial pose to start the motion from there
    std::array<double, 16> initial_pose = robot.readOnce().O_T_EE;

    double time(0.0);
    double run_time(20.0);

    // define callback function to send Cartesian pose goals to get inverse kinematics solved
    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
        cartesian_pose_callback = [&](const franka::RobotState& /*state*/,
                                      franka::Duration period) -> franka::CartesianPose {
      // update time
      time += period.s();
      if (time > run_time + acceleration_time) {
        running = false;
        return franka::Stop;
      }
      // compute Cartesian velocity
      if (vel_current < vel_max && time < run_time) {
        vel_current += period.s() * std::fabs(vel_max / acceleration_time);
      }
      if (vel_current > 0.0 && time > run_time) {
        vel_current -= period.s() * std::fabs(vel_max / acceleration_time);
      }
      vel_current = std::fmax(vel_current, 0.0);
      vel_current = std::fmin(vel_current, vel_max);

      // compute new angle for our circular trajectory
      angle += period.s() * vel_current / std::fabs(radius);
      if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
      }

      // compute relative y and z positions of desired pose
      double delta_y = radius * (1 - std::cos(angle));
      double delta_z = radius * std::sin(angle);
      std::array<double, 16> pose_desired = initial_pose;
      pose_desired[13] += delta_y;
      pose_desired[14] += delta_z;

      // send desired pose
      return pose_desired;
    };

    // define callback for the joint torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [&](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // stop moving when not running anymore
      if (!running) {
        return franka::Stop;
      }

      // read current coriolis terms from model
      std::array<double, 7> coriolis = model.coriolis(
          state, {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});

      // compute torque command from joint impedance control law
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d
      // with one time step delay
      std::array<double, 7> tau_d = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      for (size_t i = 0; i < 7; ++i) {
        tau_d[i] =
            k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
      }

      // update data to print
      if (torques_mutex.try_lock()) {
        robot_state = state;
        tau_d_last = tau_d;
        gravity = model.gravity(state, 0.0, {{0.0, 0.0, 0.0}});
        torques_mutex.unlock();
      }

      // send torque command
      return tau_d;
    };

    // start real-time control loop
    robot.control(cartesian_pose_callback, impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // stop all threads when an exception was thrown
    running = false;
    // print exception
    std::cout << ex.what() << std::endl;
  }
  print_thread.join();
  return 0;
}
