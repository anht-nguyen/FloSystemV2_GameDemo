################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.12)
project(flo_humanoid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    message_generation
    dynamixel_sdk
    message_runtime
    rospack
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
    SetJointPosition.msg
    SetJointPositions.msg
    SetArmsJointPositions.msg
)

add_service_files(
  FILES
    GetJointPosition.srv
    GetJointPositions.srv
    GetArmsJointPositions.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
    message_runtime
    message_generation
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(read_write_joint_node src/readWriteJoint.cpp)
add_dependencies(read_write_joint_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_joint_node ${catkin_LIBRARIES})

add_executable(read_write_arm_node src/readWriteArm.cpp)
add_dependencies(read_write_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_arm_node ${catkin_LIBRARIES})

add_executable(read_write_arms_node src/readWriteArms.cpp)
add_dependencies(read_write_arms_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_arms_node ${catkin_LIBRARIES})

add_executable(publish_motor_positions src/publish_motor_positions.cpp)
add_dependencies(publish_motor_positions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_motor_positions ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES})

add_executable(subscribe_motor_positions src/subscribe_motor_positions.cpp)
add_dependencies(subscribe_motor_positions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribe_motor_positions ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS
  read_write_arm_node
  read_write_joint_node
  read_write_arms_node
  publish_motor_positions
  subscribe_motor_positions
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/joint_state_to_dual_arm_motors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)