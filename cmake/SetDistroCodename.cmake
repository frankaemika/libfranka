# Use set_distro_codename to get the current distro codename.
# lsb_release is required to be used.
#
# USAGE: set_distro_codename(<output_variable>)
#
# ARGUMENTS: output_variable variable that will contain the distro codename e.g. focal
#
# EXAMPLE: 
# set_distro_codename(LSB_RELEASE)
#
function(set_distro_codename LSB_DISTRO_CODENAME)
    find_program(LSB_RELEASE_EXEC lsb_release REQUIRED)
    if(NOT LSB_RELEASE_EXEC)
        message(FATAL_ERROR "cannot execute set_distro_codename, lsb_release not found.\napt install lsb-release")
    endif()
    execute_process(COMMAND ${LSB_RELEASE_EXEC} -sc OUTPUT_VARIABLE LOCAL_SCOPE_LSB_DISTRO_CODENAME OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(${LSB_DISTRO_CODENAME} ${LOCAL_SCOPE_LSB_DISTRO_CODENAME} PARENT_SCOPE)
endfunction()