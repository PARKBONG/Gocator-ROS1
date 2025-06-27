cmake_minimum_required(VERSION 3.0.2)
project(gocator_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

add_executable(read_depth src/read_depth.cpp)
target_link_libraries(read_depth ${catkin_LIBRARIES} GoSdk kApi)
