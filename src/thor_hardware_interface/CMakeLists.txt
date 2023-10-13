cmake_minimum_required(VERSION 3.0.2)
project(thor_hardware_interface)

# Find catkin, libserial and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  control_msgs
)

# Declare a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp hardware_interface controller_manager control_msgs
)

# Include directories
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  /usr/include/libserial
)

# Declare a C++ library
add_library(${PROJECT_NAME} src/thor_hardware_interface.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  /usr/lib/aarch64-linux-gnu/libserial.so
)

# Add an executable
add_executable(${PROJECT_NAME}_node src/thor_hardware_interface_node.cpp)

# Link the executable to the necessary libs
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  /usr/lib/aarch64-linux-gnu/libserial.so
)
