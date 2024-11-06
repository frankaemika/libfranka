# Try to find fmt package. If available, take it. Otherwise, try to download.
find_package(fmt QUIET)
if(NOT fmt_FOUND)
  message(STATUS "fmt not found. Fetching fmt...")
  include(FetchContent)

  FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG        11.0.2)
  FetchContent_GetProperties(fmt)
  if(NOT fmt_POPULATED)
      FetchContent_Populate(fmt)
      add_subdirectory(${fmt_SOURCE_DIR} ${fmt_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()

  set_target_properties(fmt PROPERTIES POSITION_INDEPENDENT_CODE ON)

  if(NOT fmt_POPULATED)
    message(FATAL_ERROR "Failed to fetch fmt. Please install via 'apt install libfmt-dev' or visit https://fmt.dev/11.0/get-started/")
  endif()
else()
  message(STATUS "Found fmt: ${fmt_VERSION}")
endif()
