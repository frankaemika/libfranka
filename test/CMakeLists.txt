## Google Test
set(GTEST_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/3rdparty/googletest)
set(GTEST_INCLUDE_DIR ${GTEST_INSTALL_PREFIX}/include)
set(GTEST_LIBRARY_DIR ${GTEST_INSTALL_PREFIX}/lib)
set(GTEST_VERSION 1.8.0)

# Google Test uses pthreads if available, so we need to link that as well
find_package(Threads)

include(ExternalProject)
ExternalProject_Add(googletest
  URL https://github.com/google/googletest/archive/release-${GTEST_VERSION}.tar.gz
  URL_HASH SHA256=58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=${GTEST_INSTALL_PREFIX}
  DOWNLOAD_NAME googletest-${GTEST_VERSION}.tar.gz
  DOWNLOAD_DIR ${THIRDPARTY_SOURCES_DIR}
  SOURCE_DIR ${THIRDPARTY_SOURCES_DIR}/googletest
  BINARY_DIR ${CMAKE_BINARY_DIR}/googletest
)

set(GTEST_LIBRARIES gtest gtest_main)
foreach(library ${GTEST_LIBRARIES})
  add_library(${library} STATIC IMPORTED)

  # Workaround for https://gitlab.kitware.com/cmake/cmake/issues/15052
  file(MAKE_DIRECTORY ${GTEST_INCLUDE_DIR})

  set_target_properties(${library} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${GTEST_INCLUDE_DIR}
    INTERFACE_LINK_LIBRARIES Threads::Threads
    IMPORTED_LOCATION ${GTEST_LIBRARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}${library}${CMAKE_STATIC_LIBRARY_SUFFIX}
  )
  add_dependencies(${library} googletest)
endforeach()

## Test runner
set(TEST_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test_results)
set(TEST_DEPENDENCIES franka ${GTEST_LIBRARIES})

add_executable(run_all_tests
  robot_test.cpp
)
target_include_directories(run_all_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(run_all_tests ${TEST_DEPENDENCIES})

add_test(Default run_all_tests --gtest_output=xml:${TEST_OUTPUT_DIR}/default.xml)

if(BUILD_COVERAGE)
  message(STATUS "Coverage enabled; not adding sanitizer targets.")
  return()
endif()

## Sanitizers
include(CheckCXXCompilerFlag)
get_target_property(TESTS run_all_tests SOURCES)

set(ASan_FLAGS -g -O1 -fsanitize=address)
set(UBSan_FLAGS -g -O1 -fsanitize=undefined)
set(TSan_FLAGS -g -O1 -fsanitize=thread)
set(SANITIZERS ASan UBSan TSan)

foreach(sanitizer ${SANITIZERS})
  set(sanitizer_flags "${${sanitizer}_FLAGS}")

  set(CMAKE_REQUIRED_LIBRARIES "${sanitizer_flags}")
  check_cxx_compiler_flag("${sanitizer_flags}" HAVE_${sanitizer})
  unset(CMAKE_REQUIRED_LIBRARIES)

  if(HAVE_${sanitizer})
    string(TOLOWER "${sanitizer}" sanitizer_lc)
    set(target_name "run_all_tests_${sanitizer_lc}")

    add_executable(${target_name} ${TESTS})
    target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${target_name} ${TEST_DEPENDENCIES} ${sanitizer_flags})
    target_compile_options(${target_name} PRIVATE ${sanitizer_flags})
    add_test(NAME ${sanitizer}
      COMMAND ${target_name} --gtest_output=xml:${TEST_OUTPUT_DIR}/${sanitizer_lc}.xml
    )
  endif()
endforeach()

## Valgrind
find_program(VALGRIND_PROG valgrind)
if(VALGRIND_PROG)
  add_test(NAME Valgrind
    COMMAND ${VALGRIND_PROG} ./run_all_tests --gtest_output=xml:${TEST_OUTPUT_DIR}/valgrind.xml
  )
endif()
