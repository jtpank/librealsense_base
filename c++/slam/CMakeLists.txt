cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
project(slam)


file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

#RealSense2
find_package(realsense2 REQUIRED)

add_executable(slam ${PROJECT_SOURCE_DIR}/src/ir.cpp)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("building for aarch64")
    find_package( OpenCV 4.0.0 REQUIRED )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries(slam ${OpenCV_LIBS} realsense2)
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("building for x86_64")
endif()

#OpenGL
#find_package(OpenGL REQUIRED)

#GLUT
#find_package(GLUT REQUIRED)



# Link stage
#target_link_libraries(slam ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} realsense2)
# target_link_libraries(slam PUBLIC opengl)
# target_include_directories(slam PUBLIC include)
