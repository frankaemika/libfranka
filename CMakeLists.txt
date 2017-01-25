cmake_minimum_required(VERSION 3.4)
project(libfranka
  VERSION 0.0.1
  LANGUAGES CXX
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
   CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  add_compile_options(/Wall)
endif()

find_package(Boost REQUIRED COMPONENTS system)

set(THIRDPARTY_SOURCES_DIR "${CMAKE_SOURCE_DIR}/3rdparty" CACHE PATH
  "Directory for third-party sources")

option(BUILD_COVERAGE "Build with code coverage" OFF)
if(BUILD_COVERAGE)
  find_program(LCOV_PROG lcov)
  if(NOT LCOV_PROG)
    message(FATAL_ERROR "lcov not found.")
  endif()

  find_program(GENHTML_PROG genhtml)
  if(NOT GENHTML_PROG)
    message(FATAL_ERROR "genhtml not found.")
  endif()

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")

  set(COVERAGE_REPORT_DIR "coverage")
  set(COVERAGE_FILE "run_tests.coverage")
  set(COVERAGE_CLEANED_FILE "run_tests-cleaned.coverage")

  # Adjusted from https://github.com/bilke/cmake-modules/blob/master/CodeCoverage.cmake
  add_custom_target(coverage
    COMMAND ${LCOV_PROG} --directory . --zerocounters
    COMMAND ${CMAKE_CTEST_COMMAND} .
    COMMAND ${LCOV_PROG} --directory . --capture --output-file ${COVERAGE_FILE}
    COMMAND ${LCOV_PROG} --remove ${COVERAGE_FILE} 'test/*' '/usr/*' --output-file ${COVERAGE_CLEANED_FILE}
    COMMAND ${GENHTML_PROG} -o ${COVERAGE_REPORT_DIR} ${COVERAGE_CLEANED_FILE}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS run_all_tests
    COMMENT "Generating code coverage"
  )
endif()

## Library
set(LIB_DEPENDENCIES ${GTEST_LIBRARIES})

add_library(franka SHARED
  src/robot.cpp src/robot_state.cpp)
set_target_properties(franka PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/*.h src/*.h)

target_include_directories(franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(franka PRIVATE src)

target_link_libraries(franka Boost::boost Boost::system)

## Installation
include(GNUInstallDirs)
install(TARGETS franka
  EXPORT FrankaConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # Install .dlls into bin/ directory on Windows
)
install(EXPORT FrankaConfig DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Franka)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export targets into build directory to allow usage without installation
export(TARGETS franka FILE FrankaConfig.cmake)

## Subprojects
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_EXAMPLES "Build example code" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

option(BUILD_DOCUMENTATION "Build documentation" ON)
if(BUILD_DOCUMENTATION)
  add_subdirectory(doc)
endif()

## Packaging
set(CPACK_PACKAGE_VENDOR "FRANKA EMIKA")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION ${franka_VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Debian
find_program(DPKG_PROG dpkg DOC "'dpkg' executable")
if(DPKG_PROG)
  execute_process(COMMAND ${DPKG_PROG} --print-architecture
    OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  # Change system name to use the correct architecture in file name
  set(CPACK_SYSTEM_NAME ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE})
endif()
# Debian versions require a dash
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}-1)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FRANKA EMIKA Research Interface Team")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")

include(CPack)

## Tools
find_program(CLANG_FORMAT_PROG clang-format DOC "'clang-format' executable")
if(CLANG_FORMAT_PROG)
  add_custom_target(format
    COMMAND ${CLANG_FORMAT_PROG} -i ${SOURCES} ${HEADERS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Formatting source code with clang-format"
    VERBATIM
  )
endif()
find_program(CLANG_TIDY_PROG clang-tidy DOC "'clang-tidy' executable")
if(CLANG_TIDY_PROG)
  add_custom_target(tidy
    COMMAND ${CLANG_TIDY_PROG} -p=${CMAKE_CURRENT_BINARY_DIR} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS franka
    COMMENT "Running clang-tidy"
    VERBATIM
  )
endif()
