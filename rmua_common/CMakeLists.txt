project(rmua_common)
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
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rmua_common
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)
set(COMMON_SRC
  src/math/vec2d.cpp
  src/math/polygon2d.cpp
  src/math/math_utils.cpp
  src/math/line_segment2d.cpp
  src/math/box2d.cpp
  src/math/aabox2d.cpp

  src/visualization/visualization.cpp
  src/visualization/axis_picture.cpp
  
  src/helper.cpp
)

add_library(rmua_common
  ${COMMON_SRC}
)
target_link_libraries(rmua_common
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)