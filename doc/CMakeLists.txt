find_program(DOXYGEN_PROG doxygen)
if(NOT DOXYGEN_PROG)
  message(FATAL_ERROR "Doxygen not found.")
endif()

set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
set(DOXYGEN_EXAMPLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../examples)
set(DOXYGEN_SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include ${DOXYGEN_EXAMPLE_PATH})
string(REPLACE ";" " " DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS}")
if(STRICT)
  set(DOXYGEN_WARN_AS_ERROR "YES")
else()
  set(DOXYGEN_WARN_AS_ERROR "NO")
endif()

# Create the directory for the output and configure the Doxyfile
set(DOXYGEN_OUTPUT_PATH "docs/${PROJECT_VERSION}")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_OUTPUT_PATH})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${DOXYFILE} @ONLY)

add_custom_target(doc ALL
  COMMAND ${DOXYGEN_PROG} ${DOXYFILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating documentation with Doxygen"
  VERBATIM
)

include(GNUInstallDirs)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${CMAKE_INSTALL_DOCDIR})
