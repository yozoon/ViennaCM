cmake_minimum_required(VERSION 3.4)

project(
    "ViennaCM"
    VERSION 0.0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD "17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ViennaLS REQUIRED PATHS ${ViennaLS_DIR})
find_package(ViennaRay REQUIRED PATHS ${ViennRay_DIR})

#add_executable(lt src/layerThickness.cpp)
include_directories(lt PUBLIC include)
#target_include_directories(lt PUBLIC ${VIENNALS_INCLUDE_DIRS})
#target_link_libraries(lt ${VIENNALS_LIBRARIES})

#################################################
# BUILD EXAMPLES
#################################################
option(VIENNACM_BUILD_EXAMPLES "Build examples." OFF)
if(VIENNACM_BUILD_EXAMPLES)
  add_subdirectory(Examples)
endif(VIENNACM_BUILD_EXAMPLES)
