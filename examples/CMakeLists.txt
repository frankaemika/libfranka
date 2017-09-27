cmake_minimum_required(VERSION 3.0)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)

set(EXAMPLES
  echo_robot_state
  execute_trajectory
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
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

option(BUILD_EIGEN3_EXAMPLES "Build examples that require Eigen3" ON)
if(BUILD_EIGEN3_EXAMPLES)
  find_package(Eigen3 REQUIRED)

  set(EIGEN_EXAMPLES
    cartesian_impedance_control
    force_control
  )

  foreach(example ${EIGEN_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} Franka::Franka Eigen3::Eigen3)
  endforeach()
endif()

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
