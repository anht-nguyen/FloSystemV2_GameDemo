cmake_minimum_required(VERSION 2.8.3)
project(flo_core_defs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  flo_humanoid
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GameAction.msg
  GameCommand.msg
  GameCommandOptions.msg
  GameDef.msg
  GameState.msg
  StepDef.msg
  PoseScore.msg 
  Emotion.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPoseID.srv
  GetPoseSeqID.srv
  SetFace.srv
  SetPose.srv
  SetUtterance.srv
)

add_action_files(
  FILES
    SimonCmd.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  flo_humanoid
)




###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    actionlib_msgs
    flo_humanoid
)




###########
## Build ##
###########

## Specify additional locations of header files
include_directories(  
  ${catkin_INCLUDE_DIRS}
)