cmake_minimum_required(VERSION 3.4)

project("GeometryToGraph")

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${VIENNACM_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${VIENNACM_LIBRARIES})
if(WITH_GZIP)
    target_link_libraries(${PROJECT_NAME} PRIVATE ZLIB::ZLIB)
endif(WITH_GZIP)

# Copy files
file(COPY first.lvst DESTINATION ${PROJECT_BINARY_DIR})

# Add to example target
add_dependencies(buildExamples ${PROJECT_NAME})
