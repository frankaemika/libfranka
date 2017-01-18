cmake_minimum_required(VERSION 3.0)
project(franka-examples CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Standalone build of examples
  find_package(Franka REQUIRED)
endif()

foreach(sample echo_robot_state)
  add_executable(${sample} ${sample}.cpp)
  target_link_libraries(${sample} franka)
endforeach()
