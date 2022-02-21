cmake_minimum_required(VERSION 3.4)

project("MoveSurface")

find_package(ViennaHRLE REQUIRED)
find_package(ViennaLS REQUIRED)
find_package(VTK REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${VIENNALS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${VIENNALS_LIBRARIES})

file(COPY substrate.vtp DESTINATION ${PROJECT_BINARY_DIR})

# Add to example target
add_dependencies(buildExamples ${PROJECT_NAME})