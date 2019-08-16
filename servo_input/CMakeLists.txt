cmake_minimum_required(VERSION 2.8.3)
project(servo_input)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/servo_input
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
