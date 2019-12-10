cmake_minimum_required(VERSION 2.8.3)
project(mocap2odom)

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_stl_containers
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf
  DEPENDS Boost EIGEN3 YAML_CPP tf
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

# Vicon VRPN to odom message
add_executable(Vicon2odom
    src/Vicon2odom.cpp
)
add_dependencies(Vicon2odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Vicon2odom
  ${catkin_LIBRARIES}
)
# Optitrack VRPN to odom message
add_executable(OptiTrack2odom
    src/OptiTrack2odom.cpp
)
add_dependencies(OptiTrack2odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(OptiTrack2odom
	${catkin_LIBRARIES}
)
# odom message to transform
add_executable(odom2transform
    src/odom2transform.cpp
)
add_dependencies(odom2transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom2transform
        ${catkin_LIBRARIES}
)
#############
## Install ##
#############

