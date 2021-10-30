cmake_minimum_required(VERSION 3.0.2)
project(axis6)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
  message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf visualization_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/ SRC)

add_executable(robot_sim apps/robot_sim.cc
  ${SRC}
)

target_link_libraries(robot_sim
  ${catkin_LIBRARIES}
  pthread
)

add_executable(gazebo_sim apps/gazebo_sim.cc
  ${SRC}
)

target_link_libraries(gazebo_sim
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS robot_sim gazebo_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
