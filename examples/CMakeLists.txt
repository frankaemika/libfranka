cmake_minimum_required(VERSION 3.0)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(EXAMPLES
  cartesian_impedance_control
  echo_robot_state
  execute_trajectory
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_positions
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

target_link_libraries(cartesian_impedance_control Eigen3::Eigen3)
target_link_libraries(force_control Eigen3::Eigen3)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
