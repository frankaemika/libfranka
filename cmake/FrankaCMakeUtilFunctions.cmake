# Starting at SEARCH_DIR, look up directory hierarchy for directory containing
# NAME. SEARCH_DIR is a directory. Stop at the first parent directory containing
# a folder NAME, and return the directory in the parent scope in
# FOUND_DOMINATING_DIR.  Return empty if not found.
#
# USAGE: locate_dominating_dir( NAME <dir name> SEARCH_DIR <dir path> )
#
# ARGUMENTS: NAME name of the directory to be found SEARCH_DIR directory path
# where the search starts
#
# OUTPUT VARIABLES: FOUND_DOMINATING_DIR path to the directory where NAME is
# first found
#
# EXAMPLE: locate_dominating_dir( NAME               .git SEARCH_URL
# ${CMAKE_CURRENT_LIST_DIR} )
#
function(locate_dominating_dir)
  cmake_parse_arguments(ARG "" "NAME;SEARCH_DIR" "" ${ARGN})
  if(IS_DIRECTORY ${ARG_SEARCH_DIR}/${ARG_NAME})
    set(FOUND_DOMINATING_DIR
        "${ARG_SEARCH_DIR}"
        PARENT_SCOPE)
  elseif(EXISTS ${ARG_SEARCH_DIR}/..)
    locate_dominating_dir(NAME ${ARG_NAME} SEARCH_DIR ${ARG_SEARCH_DIR}/..)
    set(FOUND_DOMINATING_DIR
        ${FOUND_DOMINATING_DIR}
        PARENT_SCOPE)
  endif()
endfunction()

# Return the git repository name of the calling CMakeLists.txt.
#
# USAGE: get_git_repo_name()
#
# OUTPUT VARIABLES: GIT_REPO_NAME git repository name of the calling
# CMakeLists.txt
#
function(get_git_repo_name)
  if(NOT GIT_FOUND
     OR NOT BASENAME_PROG
     OR NOT XARGS_PROG)
    return()
  endif()
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.url
    COMMAND xargs ${BASENAME_PROG} -s .git
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE GIT_REPO_NAME
    RESULT_VARIABLE GIT_GET_NAME_RESULT
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT GIT_GET_NAME_RESULT EQUAL "0")
    unset(GIT_REPO_NAME)
    message(
      WARNING
        "${CMAKE_CURRENT_LIST_FILE} is probably not inside git repository.")
  endif()
  set(GIT_REPO_NAME
      ${GIT_REPO_NAME}
      PARENT_SCOPE)
endfunction()
