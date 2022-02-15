cmake_minimum_required(VERSION 3.4)

project("SurfaceMapping")

find_package(ViennaHRLE REQUIRED)
find_package(ViennaLS REQUIRED)
find_package(VTK REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${VIENNALS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${VIENNALS_LIBRARIES})

# Copy files
configure_file(substrate.vtp ${PROJECT_BINARY_DIR}/substrate.vtp COPYONLY)
configure_file(deposited.vtp ${PROJECT_BINARY_DIR}/deposited.vtp COPYONLY)

# Add to example target
add_dependencies(buildExamples ${PROJECT_NAME})
add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)