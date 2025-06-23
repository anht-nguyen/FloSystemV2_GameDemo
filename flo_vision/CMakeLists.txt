cmake_minimum_required(VERSION 3.0.2)
project(flo_vision)

###############################################################################
# 1. Catkin & system packages
###############################################################################
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

# Python helper – lets `setup.py` install modules and scripts
catkin_python_setup()

###############################################################################
# 2. Catkin metadata
###############################################################################
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport rospy sensor_msgs std_msgs
  # Non-catkin Python libraries (mediapipe, opencv-python) are listed in package.xml only
)

###############################################################################
# 3. Include paths
###############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###############################################################################
# 4. Install the Python nodes & helper modules
###############################################################################
# All *.py under scripts/ (or another folder) will be made executable and put
# in the correct ${CATKIN_PACKAGE_BIN_DESTINATION}.

# install ONLY the executable script here
catkin_install_python(
  PROGRAMS scripts/arm_hand_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


###############################################################################
# 5. Install launch files, config, etc. (optional but recommended)
###############################################################################
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

###############################################################################
# 6. Tests (optional)
###############################################################################
# catkin_add_nosetests(test)
