cmake_minimum_required(VERSION 2.8.3)
project(filter_pointcloud)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs 
)


add_library(
    filter
    src/filter_pointcloud.hpp
    src/filter_pointcloud.cpp
)

target_include_directories(filter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" )

include_directories(
  ${catkin_INCLUDE_DIRS}
  filter
)

add_executable(main src/main.cpp)
add_dependencies(main ${catkin_EXPORTED_TARGETS})
target_link_libraries(main PRIVATE ${catkin_LIBRARIES} filter)