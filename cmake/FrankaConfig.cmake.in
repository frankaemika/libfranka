get_filename_component(Franka_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(NOT TARGET Franka::Franka)
  include("${Franka_CMAKE_DIR}/FrankaTargets.cmake")
endif()

# It is recommended to use target_link_libraries(<target> Franka::Franka) instead.
set(Franka_LIBRARIES Franka::Franka)
get_target_property(Franka_INCLUDE_DIRS Franka::Franka INTERFACE_INCLUDE_DIRECTORIES)
