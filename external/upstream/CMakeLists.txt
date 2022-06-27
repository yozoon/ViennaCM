## external project base directory for all projects in this hierarchy
set_property(DIRECTORY PROPERTY EP_BASE ${DEPENDENCIES_DIR})

# build msgpack if no path was specified
if(NOT DEFINED msgpack_DIR)
    add_subdirectory(msgpack)
    message(STATUS "msgpack_DIR: ${msgpack_DIR}")
endif()