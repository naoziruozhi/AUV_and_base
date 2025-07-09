cmake_minimum_required(VERSION 3.0.2)
project(mav_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 只安装可执行脚本
catkin_install_python(PROGRAMS
  scripts/mav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

