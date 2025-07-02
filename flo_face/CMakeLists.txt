cmake_minimum_required(VERSION 3.0.2)
project(flo_face)

##############################################################################
# Dependencies
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  flo_core_defs        # custom message package used by Emotion.msg
  message_generation
  rospack
)

##############################################################################
# Catkin package
##############################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs flo_core_defs message_generation rospack
)

##############################################################################
# Install Python executables
##############################################################################
install(PROGRAMS
  scripts/face_com_manager.py
  scripts/test/demo_face.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##############################################################################
# Install Python libraries (serial_coms)
##############################################################################
install(DIRECTORY scripts/serial_coms/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/serial_coms
  FILES_MATCHING PATTERN "*.py"
)

##############################################################################
# Install data (faces.json) and launch files
##############################################################################
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  FILES_MATCHING PATTERN "*.json"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

##############################################################################
# (No C++ targets to build for this package)
##############################################################################

