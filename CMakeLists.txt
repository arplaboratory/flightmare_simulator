cmake_minimum_required(VERSION 2.8.3)
project(flightmare_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")

find_package(catkin REQUIRED COMPONENTS roscpp quadrotor_msgs nav_msgs sensor_msgs flightlib tf2_ros image_transport)
find_package(Eigen3 REQUIRED)
find_package(catkin_simple)

catkin_simple()

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp quadrotor_msgs nav_msgs sensor_msgs tf2_ros  flightlib image_transport
  DEPENDS EIGEN3)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} stdc++fs zmq zmqpp)
add_dependencies(quadrotor_simulator_so3 ${OpenCV_LIBRARIES} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(quadrotor_simulator_trpy src/quadrotor_simulator_trpy.cpp)
#target_link_libraries(quadrotor_simulator_trpy quadrotor_dynamics ${catkin_LIBRARIES}            #stdc++fs
#    zmq
#zmqpp)
#add_dependencies(quadrotor_simulator_trpy ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#install(TARGETS
#    quadrotor_dynamics
#    quadrotor_simulator_so3
#    quadrotor_simulator_trpy
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
