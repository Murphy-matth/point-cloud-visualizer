cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(vis)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


SET(CMAKE_CXX_FLAGS  "-Wdeprecated-declarations" )
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp visualizer.cc)

add_executable(vis ${SOURCE_FILES})
target_link_libraries(vis ${PCL_LIBRARIES})