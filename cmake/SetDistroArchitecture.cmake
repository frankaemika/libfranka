# Use set_distro_architecture to get the current architecture.
# dpkg is required to be used.
#
# USAGE: set_distro_architecture(<output_variable>)
#
# ARGUMENTS: output_variable variable that will contain the architecture e.g. amd64
#
# EXAMPLE: 
# set_distro_architecture(ARCHITECTURE)
#
function(set_distro_architecture DISTRO_ARCHITECTURE)
    find_program(DPKG_EXEC dpkg REQUIRED)
    if(NOT DPKG_EXEC)
        message(FATAL_ERROR "cannot execute set_distro_architecture, dpkg not found.\napt install dpkg")
    endif()
    execute_process(COMMAND ${DPKG_EXEC} --print-architecture OUTPUT_VARIABLE LOCAL_SCOPE_DISTRO_ARCHITECTURE OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(${DISTRO_ARCHITECTURE} ${LOCAL_SCOPE_DISTRO_ARCHITECTURE} PARENT_SCOPE)
endfunction()