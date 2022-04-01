# Use set_version_from_git to get the current version and tag from git.
# If the current commit is tagged the git tag is returned as version.
# If not, the latest tag is returned. If no tag exists in the commits history, 0.1.0 is returned.
# This works only if the latest git tag is a valid version in the following
# format, or it has never been set:
# ${MAJOR}.${MINOR}.${PATCH}
#
# USAGE: set_version_from_git(<output_version> <output_tag>)
#
# ARGUMENTS: output_version variable that will contain the version
#            output_tag variable that will contain the tag
#
# EXAMPLE: 
# set_version_from_git(PROJECT_VERSION PROJECT_TAG)
# project(<project_name> VERSION ${PROJECT_VERSION})
#
function(set_version_from_git VERSION TAG)
    execute_process(COMMAND git describe --tags
                    OUTPUT_VARIABLE LOCAL_SCOPE_TAG
                    ERROR_QUIET)
    # separate the version from the commit short hash
    if(LOCAL_SCOPE_TAG)
        string(STRIP ${LOCAL_SCOPE_TAG} LOCAL_SCOPE_TAG)
        string(FIND ${LOCAL_SCOPE_TAG} "-" FIND_INDEX)
        string(SUBSTRING ${LOCAL_SCOPE_TAG} 0 ${FIND_INDEX} LOCAL_SCOPE_VERSION)
    else()
        set(LOCAL_SCOPE_VERSION "0.1.0")
    endif() 
    
    set(${VERSION} ${LOCAL_SCOPE_VERSION} PARENT_SCOPE)
    set(${TAG} ${LOCAL_SCOPE_TAG} PARENT_SCOPE)
endfunction()