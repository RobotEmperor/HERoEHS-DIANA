################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(diana_online_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  heroehs_math
  heroehs_online_walking_pattern_generator
  heroehs_pd_balance_controller
  diana_kinematics_dynamics
  diana_online_walking_module_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diana_online_walking_module
  CATKIN_DEPENDS roscpp robotis_math robotis_framework_common heroehs_math heroehs_online_walking_pattern_generator heroehs_pd_balance_controller diana_kinematics_dynamics
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(diana_online_walking_module
  src/diana_online_walking.cpp
  src/online_walking_module.cpp
)

add_dependencies(diana_online_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(diana_online_walking_module
  ${catkin_INCLUDE_DIRS}
)
