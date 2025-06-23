cmake_minimum_required(VERSION 2.8.3)
project(flo_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  flo_core_defs
  flo_humanoid
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs flo_core_defs flo_humanoid
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Example: Uncomment and edit the following lines if you have source files to build
# add_executable(example_node src/example_node.cpp)
# add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(example_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install launch, config, and scripts directories
install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

