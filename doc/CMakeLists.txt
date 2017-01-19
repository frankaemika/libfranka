find_program(DOXYGEN_PROG doxygen)
if(NOT DOXYGEN_PROG)
  message(FATAL_ERROR "Doxygen not found.")
endif()

set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
set(SOURCE_DIRS "${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${DOXYFILE} @ONLY)

add_custom_target(doc ALL
  COMMAND ${DOXYGEN_PROG} ${DOXYFILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating documentation with Doxygen"
  VERBATIM
)

include(GNUInstallDirs)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${CMAKE_INSTALL_DOCDIR})
