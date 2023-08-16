cmake_minimum_required(VERSION 3.0.2)
project(thor_control)

# Find Eigen3
find_package(Eigen3 REQUIRED)
# Find Assimp
find_package(assimp REQUIRED)

# Find catkin and the required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosparam_shortcuts
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
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
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(thor_control src/thor_control.cpp src/pick_place_task.cpp)
add_executable(thor_control_legacy src/thor_control_legacy.cpp)

# Declare a C++ library
add_library(mesh_loader src/mesh_loader.cpp)

# Specify libraries to link the executable with
target_link_libraries(thor_control
  ${catkin_LIBRARIES}
)
target_link_libraries(thor_control_legacy
  ${catkin_LIBRARIES}
)
target_link_libraries(thor_control
  mesh_loader
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)
