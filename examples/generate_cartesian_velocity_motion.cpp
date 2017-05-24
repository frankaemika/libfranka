#include <cmath>
#include <iostream>

#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_cartesian_velocity_motion <robot-hostname>"
              << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop
    // Set the cartesian impedance:
    robot.setCartesianImpedance({{1500, 1500, 1500, 150, 150, 150}});

    // Set the joint impedance:
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior:
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{
        {30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{
        {40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{
        {30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{
        {40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration,
        upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
        upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
        upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
        upper_force_thresholds_nominal);

    // Set a dynamic load:
    double load_mass = 0.1;
    std::array<double, 3> load_translation{{0.0, 0.0, 0.0}};
    std::array<double, 9> load_inertia{
        {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}};
    robot.setLoad(load_mass, load_translation, load_inertia);

    double time_max = 4.0;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
    double time = 0.0;
    robot.control(
        [=, &time](const franka::RobotState&) -> franka::CartesianVelocities {
          double cycle = std::floor(
              pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double v = cycle * v_max / 2.0 *
                     (1.0 - std::cos(2.0 * M_PI / time_max * time));
          double v_x = std::cos(angle) * v;
          double v_z = -std::sin(angle) * v;

          time += 0.001;
          if (time > 2 * time_max) {
            std::cout << std::endl
                      << "Finished motion, shutting down example" << std::endl;
            return franka::Stop;
          }
          return {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
        });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
