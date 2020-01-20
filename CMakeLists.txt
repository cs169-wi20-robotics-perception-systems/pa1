cmake_minimum_required(VERSION 2.8.3)
project(pa1)

find_package(catkin REQUIRED
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/move_rosbot_forward.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/plot_rosbot_odom.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
