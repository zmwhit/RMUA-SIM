project(rmua_simulator)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
# SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  tf
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  tinyspline_ros
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp)
find_package(rmua_common REQUIRED)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rmua_common_INCLUDE_DIRS}
  include
)
add_executable(simulator
  src/simulator.cpp
  src/chassis_kinematic.cpp
  src/gimbal_kinematic.cpp
)
target_link_libraries(simulator
  rmua_common
  ${catkin_LIBRARIES}
)

add_executable(map_manager
  src/map_manager.cpp
)
target_link_libraries(map_manager
  yaml-cpp  
  rmua_common
  ${catkin_LIBRARIES}
)



