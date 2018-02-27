################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(diana_kinematics_dynamics)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_math
  cmake_modules
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
  INCLUDE_DIRS include
  LIBRARIES diana_kinematics_dynamics
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(diana_kinematics_dynamics
  src/link_data.cpp
  src/kinematics_dynamics.cpp
)

add_dependencies(diana_kinematics_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(diana_kinematics_dynamics
  ${catkin_LIBRARIES}
)
