cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
project(slam)
set(ENTRY_FILE "ir.cpp")

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

add_executable(slam ${PROJECT_SOURCE_DIR}/src/${ENTRY_FILE})

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("\nbuilding for aarch64")
    #opencv
    find_package( OpenCV 4.0.0 REQUIRED )
    #RealSense2
    find_package(realsense2 REQUIRED)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries(slam ${OpenCV_LIBS} realsense2)
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("\nbuilding for x86_64")
    #RealSense2
    #windows:
    find_package(realsense2 CONFIG REQUIRED)
    #target_link_libraries(main PRIVATE realsense2::fw realsense2::rsutils realsense2::realsense2 realsense2::realsense-file)
    # opencv
    find_package( OpenCV REQUIRED )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    target_link_libraries( slam ${OpenCV_LIBS} realsense2 )
endif()

#OpenGL
#find_package(OpenGL REQUIRED)

#GLUT
#find_package(GLUT REQUIRED)



# Link stage
#target_link_libraries(slam ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} realsense2)
# target_link_libraries(slam PUBLIC opengl)
# target_include_directories(slam PUBLIC include)
