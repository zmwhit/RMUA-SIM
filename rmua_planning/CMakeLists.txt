project(rmua_planning)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
# SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tinyspline_ros
)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(rmua_common REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${rmua_common_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}

  ${osqp_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)
set(EXTERNAL_LIBS
  rmua_common
  ${PYTHON_LIBRARIES}

  osqp::osqp
  OsqpEigen::OsqpEigen
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)
set(SRC_LIST
  src/planning_node.cpp
  src/a_star_planner.cpp
  src/path_smoother.cpp
  src/speed_optimizer.cpp
  src/nlp_planner.cpp

  src/teb_local_planner/obstacles.cpp
  src/teb_local_planner/teb_planner.cpp
  src/teb_local_planner/teb_config.cpp
  src/teb_local_planner/timed_elastic_band.cpp
)
add_executable(planning_node
  ${SRC_LIST}
)
target_link_libraries(planning_node
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)