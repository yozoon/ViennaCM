cmake_minimum_required(VERSION 3.4)

project("KDTreeBenchmark")

find_package(ViennaHRLE REQUIRED)
find_package(ViennaLS REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${VIENNACM_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${VIENNACM_LIBRARIES})

# Add to example target
add_dependencies(buildExamples ${PROJECT_NAME})