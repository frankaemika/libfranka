cmake_minimum_required(VERSION 3.0)
project(libfranka-examples CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(TARGET franka)
  # Integrated build of examples (included by add_subdirectory())
  set(EXAMPLE_DEPENDENCIES franka)
else()
  # Standalone build of examples
  find_package(Franka REQUIRED)
  set(EXAMPLE_DEPENDENCIES Franka::franka)
endif()

set(EXAMPLES echo_robot_state
             cartesian_pose_motion_example
             cartesian_velocity_motion_example
             joint_velocity_motion_example
             joint_pose_motion_example)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} ${EXAMPLE_DEPENDENCIES})
endforeach()

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
