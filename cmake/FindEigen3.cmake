find_package(Eigen3 CONFIG)
mark_as_advanced(FORCE Eigen3_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
  FOUND_VAR Eigen3_FOUND
  REQUIRED_VARS EIGEN3_INCLUDE_DIRS
)

if(NOT TARGET Eigen3::Eigen3)
  add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
    INTERFACE_COMPILE_DEFINITIONS "${EIGEN3_DEFINITIONS}"
  )
endif()
