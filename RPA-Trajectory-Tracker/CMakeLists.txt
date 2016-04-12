cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test)

find_package(PCL 1.2 REQUIRED PATHS /usr/share NO_DEFAULT_PATH)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test src/main.cpp src/util.cpp src/trajectory.cpp src/people_detector.cpp src/downsample.cpp src/people_tracker.cpp src/cluster_data.cpp src/draw.cpp src/plane.cpp)
target_link_libraries (test ${PCL_LIBRARIES})
target_link_libraries (test ${OpenCV_LIBS})
