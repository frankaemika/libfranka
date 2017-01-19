cmake_minimum_required(VERSION 3.0)
project(libfranka-examples CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Standalone build of examples
  find_package(Franka REQUIRED)
endif()

set(EXAMPLES echo_robot_state)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} franka)
endforeach()

install(TARGETS ${EXAMPLES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
