# Override CXX standard, since Embree does not confirm to CXX20.
set(CMAKE_CXX_STANDARD 17)

# This cmake policy is for the git_submodules parameter below
cmake_policy(SET CMP0097 NEW)

externalproject_add(msgpack-external
  GIT_REPOSITORY
    https://github.com/embree/embree.git
  GIT_TAG
    cpp_master
  GIT_SUBMODULES
    ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
    -DCMAKE_CXX_EXTENSIONS=${CMAKE_CXX_EXTENSIONS}
    -DCMAKE_CXX_STANDARD_REQUIRED=${CMAKE_CXX_STANDARD_REQUIRED}
    -DCMAKE_SHARED_LINKER_FLAGS=-lstdc++
    -DMSGPACK_BUILD_DOCS=OFF
    -DMSGPACK_CXX17=ON
    -DMSGPACK_USE_BOOST=OFF
    -DMSGPACK_BUILD_TESTS=OFF
  CMAKE_CACHE_ARGS
    #-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
  USES_TERMINAL_DOWNLOAD
    1
  USES_TERMINAL_UPDATE
    1
  GIT_PROGRESS
    1
)

# Get install and build directory from external project
externalproject_get_property(msgpack-external INSTALL_DIR BINARY_DIR)

set(
  msgpack_DIR "${INSTALL_DIR}"
  CACHE PATH "Path to msgpack installation"
  FORCE
  )
set(
    msgpack_INCLUDEDIR "${msgpack_DIR}/include"
  CACHE PATH "Path to msgpack include directories"
  FORCE
  )
set(
    msgpack_LIBRARYDIR "${msgpack_DIR}/lib"
  CACHE PATH "Path to msgpack library directories"
  FORCE
  )