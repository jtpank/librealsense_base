cmake_minimum_requirements(VERSION 3.3)
set(CMAKE_CXX_STANDARD 17)
project(slam)

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/../bin")

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    # do stuff
endif()

target_link_libraries(slam PUBLIC opengl)
target_include_directories(slam PUBLIC include)543 