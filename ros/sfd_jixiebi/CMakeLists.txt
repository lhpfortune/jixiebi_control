cmake_minimum_required(VERSION 3.0.2)
project(sfd_jixiebi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  urdf
  xacro
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    urdf
    xacro
    geometry_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vision_color_detector.py
  scripts/pick_place_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)