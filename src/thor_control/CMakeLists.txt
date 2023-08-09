cmake_minimum_required(VERSION 3.0.2)
project(thor_control)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find catkin and the required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  thor_moveit
  thor_hardware_interface
  thor_description
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  thor_moveit
  thor_hardware_interface
  thor_description
)

# Specify additional locations of header files
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(thor_control src/thor_control.cpp)

# Specify libraries to link the executable with
target_link_libraries(thor_control
  ${catkin_LIBRARIES}
)

