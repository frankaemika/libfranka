# Use set_debian_package_name to get the debian package name to use in CPack.
# The naming is produced accorgingly to Franka Emika policy.
# The output name will be composed as:
# <project_name>_<package_version>-1_<distro_codename>_<distro_arch>.deb
# e.g. if <project_name> = cmake-tools
# cmake-tools_1.0.0-1_focal_all.deb     # if target focal distro on 64/32 bit arch
# or
# cmake-tools_1.0.0-1_amd64.deb         # if targets all distros but only on 64 bit arch
#
# USAGE: set_debian_package_name(<output_filename>)
#
# ARGUMENTS: output_filename output variable of the current package name
#
# EXAMPLE to set package name automatically from the current host machine: 
# ...
# set_debian_package_name(CPACK_PACKAGE_FILE_NAME)
# ...
# include(CPack)
#
# EXAMPLE to set package name manually for multiple targets: 
# ...
# set_debian_package_name(CPACK_PACKAGE_FILE_NAME DISTRO "all" ARCH "all")
# ...
# include(CPack)
#
# EXAMPLE to set package name as library for development:
# ...
# set_debian_package_name(CPACK_PACKAGE_FILE_NAME LIB ON DEV ON)
# # this will generate libcmake-tools-dev_1.0.0-1_focal_all.deb
# ...
# include(CPack)

function(set_debian_package_name PACKAGE_FILE_NAME)
    cmake_parse_arguments(ARG "LIB;DEV" "DISTRO;ARCH" "" ${ARGN})

    # get the version from git
    include(SetVersionFromGit)
    set_version_from_git(PROJECT_VERSION PROJECT_TAG)

    # use kebab-case for project naming
    string(REPLACE "_" "-" FILE_NAME ${PROJECT_NAME})

    # append lib to the name
    if(ARG_LIB)
        set(FILE_NAME "lib${FILE_NAME}")
    endif()

    # append dev to the name
    if(ARG_DEV)
        set(FILE_NAME "${FILE_NAME}-dev")
    endif()

    # append filename and version to the package name
    set(TMP_PACKAGE_FILE_NAME "${FILE_NAME}_${PROJECT_TAG}-1")

    # if distro codename differs from all then append ARG_DISTRO if specified or the host distro
    set(SUPPORTED_DISTROS "xenial" "focal" "bionic" "jammy")
    if(ARG_DISTRO IN_LIST SUPPORTED_DISTROS)
        set(TMP_PACKAGE_FILE_NAME "${TMP_PACKAGE_FILE_NAME}_${ARG_DISTRO}")
    elseif(NOT ARG_DISTRO)
        #compute distro codename automatically
        include(SetDistroCodename)
        set_distro_codename(TMP_DISTRO_CODENAME)
        set(TMP_PACKAGE_FILE_NAME "${TMP_PACKAGE_FILE_NAME}_${TMP_DISTRO_CODENAME}")
    elseif(NOT ARG_DISTRO STREQUAL "all")
        message(FATAL_ERROR "set_debian_package_name invalid argument: DISTRO must be one of ${SUPPORTED_DISTROS}")
    endif()
    
    # set archictecture name
    set(SUPPORTED_ARCHS "i386" "amd64" "armel" "armhf" "arm64" "mips" "mips64el" "mipsel" "ppc64el" "s390x" "all")
    if(ARG_ARCH IN_LIST SUPPORTED_ARCHS)
        set(TMP_DISTRO_ARCH ${ARG_ARCH})
    elseif(NOT ARG_ARCH)
        #compute distro arch automatically
        include(SetDistroArchitecture)
        set_distro_architecture(TMP_DISTRO_ARCH)
    else()
        message(FATAL_ERROR "set_debian_package_name invalid argument: ARCH must be one of ${SUPPORTED_ARCHS}")
    endif()

    # set the distro architecture
    set(${PACKAGE_FILE_NAME} "${TMP_PACKAGE_FILE_NAME}_${TMP_DISTRO_ARCH}" PARENT_SCOPE)

endfunction()
