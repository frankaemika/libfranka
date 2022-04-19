# Use set_debian_package_name to get the debian package name to use in CPack.
# The naming is produced accorgingly to the company policy.
# The output name will be composed as:
# <project_name>_<package_version>-1_<distro_codename>_<distro_arch>.deb
# e.g.
# cmake_tools_1.0.0-1-focal-all.deb
#
# USAGE: set_debian_package_name(<output_filename>)
#
# ARGUMENTS: output_filename output variable of the current package name
#
# EXAMPLE: 
# ...
# set_debian_package_name(CPACK_PACKAGE_FILE_NAME)
# ...
# include(CPack)
#
function(set_debian_package_name PACKAGE_FILE_NAME)
    include(SetDistroArchitecture)
    set_distro_architecture(DISTRO_ARCH)

    include(SetDistroCodename)
    set_distro_codename(DISTRO_CODENAME)

    include(SetVersionFromGit)
    set_version_from_git(PROJECT_VERSION PROJECT_TAG)

    if (PACKAGE_TAG)
        set(${PACKAGE_FILE_NAME} "${PROJECT_NAME}_${PACKAGE_TAG}-1_${DISTRO_CODENAME}_${DISTRO_ARCH}" PARENT_SCOPE)
    else()
        # this is a special case just if a tag in the commit history never existed
        set(ENV{GIT_DIR} ${CMAKE_SOURCE_DIR}/.git)
        execute_process(COMMAND git describe --tags --always
            OUTPUT_VARIABLE SHORT_HASH
            ERROR_QUIET)
        string(STRIP ${SHORT_HASH} SHORT_HASH)
        set(${PACKAGE_FILE_NAME} "${PROJECT_NAME}_${SHORT_HASH}-1_${DISTRO_CODENAME}_${DISTRO_ARCH}" PARENT_SCOPE)
    endif()
endfunction()
