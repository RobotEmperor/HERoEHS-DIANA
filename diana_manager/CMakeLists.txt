################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(diana_manager)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  cmake_modules
  heroehs_math
  heroehs_online_walking_pattern_generator
  heroehs_pd_balance_controller
  diana_kinematics_dynamics
  diana_online_walking_module
  diana_base_module
)

find_package(Eigen3 REQUIRED)

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
  LIBRARIES diana_soccer_manager
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(diana_soccer_manager src/diana_soccer_manager.cpp)

add_dependencies(diana_soccer_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(diana_soccer_manager
  ${catkin_LIBRARIES}
)
