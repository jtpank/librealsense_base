cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 17)
project(slam)


file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("building for aarch64")
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("building for x86_64")
endif()

#OpenGL
find_package(OpenGL REQUIRED)
#GLUT
find_package(GLUT REQUIRED)

add_executable(slam ${PROJECT_SOURCE_DIR}/src/main.cpp)

# Link stage
target_link_libraries(slam ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )
# target_link_libraries(slam PUBLIC opengl)
# target_include_directories(slam PUBLIC include)
