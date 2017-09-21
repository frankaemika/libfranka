#include <unistd.h>
#include <array>
#include <cmath>
#include <atomic>
#include <functional>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>

/**
 * @example impedance_linear_ds.cpp
 * An example showing a simple cartesian impedance controller with inertia shaping
 * that renders a spring damper system (only position)
 * where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 * Usage: ./impedance_linear_ds <robot-hostname> <stiffness>
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: ./" << argv[0]
              << " <robot-hostname>" << std::endl;
    return -1;
  }

  // initialize controller parameters
  double max_cart_vel(0.7);
  double k_d_task(30.0);
  double k_p_nullspace(100.0);
  double k_ext(0.9);
  // to compute cartesian velocities
  double filter_gain = 0.8;

  // desired task dynamics x_d_dot = A (x_equilibrium - x)
  Eigen::MatrixXd A(6,6);
  Eigen::VectorXd x_equilibrium(6);
  A.setZero();
  A.topLeftCorner(3,3) << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1;

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    /* Compute initial velocity with jacobian and set x_equilibrium to initial configuration*/
    // get jacobian
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
    Eigen::Matrix<double, 6, 1> x, dx;
    x_equilibrium << initial_state.O_T_EE[12], initial_state.O_T_EE[13], initial_state.O_T_EE[14], 0 , 0, 0;
    dx << jacobian * dq_initial;
    dx.tail(3) << 0, 0, 0; // only position

    // set collision behavior
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [&](
            const franka::RobotState& state, franka::Duration time_step) -> franka::Torques {
          /* Get robot state */
          // copy current state
          franka::RobotState robot_state(state);
          // get coriolis
          std::array<double, 7> coriolis_array = model.coriolis(
              robot_state, {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
          // get jacobian
          std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
          // get intertia
          std::array<double, 49> inertia_array = model.mass(robot_state,
                                                            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
                                                            0.0, {{0.0, 0.0, 0.0}});
          // convert to eigen
          Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
          Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
          Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
          Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
          Eigen::Map<Eigen::Matrix<double, 7, 7> > inertia(inertia_array.data());
          Eigen::Matrix<double, 6, 6> cartesian_mass;

          x.head(3) << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];

          Eigen::MatrixXd cartesian_mass_inverse((jacobian * inertia.inverse() * jacobian.transpose()));

          // Invert with SVD
          Eigen::EigenSolver<Eigen::MatrixXd> es(cartesian_mass_inverse, true);
          Eigen::VectorXd eigenvals = es.eigenvalues().real();

          // Ensure positive definiteness
          for (size_t e=1 ; e < 6 ; e++) {
            if (eigenvals(e) < 1e-3) {
              eigenvals(e) = 1e-3;
            }
          }

          cartesian_mass = es.eigenvectors().real() * eigenvals.asDiagonal().inverse()
              * es.eigenvectors().real().transpose();

          /* Exponential smoother to compute velocity */
          // Note: first time_step is 0.0!
          if (time_step.toSec() > 0.0) {
            dx = (1.0 - filter_gain) * dx + filter_gain * (jacobian * dq);
            dx.tail(3) << 0, 0, 0; // only position
          }

          /* Compute controller */
          // allocate controller variables
          Eigen::VectorXd tau_task(7);
          std::array<double, 7> tau_d = {{0, 0, 0, 0, 0, 0, 0}};

          tau_task = jacobian.transpose() * cartesian_mass * (stiffness * (x_equilibrium - x) - 2.0 * sqrt(stiffness) * dx));

          for (size_t i = 0; i < 7; ++i) {
            tau_d[i] = tau_task(i) + coriolis(i)
                - k_ext * robot_state.tau_ext_hat_filtered[i]; // reduce perceived mass at a joint level
          }
          // send torque command
          return tau_d;
        };

    // start real-time control loop
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}