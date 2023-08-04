cmake_minimum_required(VERSION 2.8.3)
project(flightmare_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")


find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs flightlib tf2_ros image_transport pcl_conversions)
find_package(Eigen3 REQUIRED)
find_package(catkin_simple)

catkin_simple()

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${zmqq_INCLUDE_DIR} )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros  flightlib image_transport  
  DEPENDS EIGEN3)

add_library(flightmare_quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 flightmare_quadrotor_dynamics ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} stdc++fs zmq zmqpp)
add_dependencies(quadrotor_simulator_so3 ${OpenCV_LIBRARIES} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
