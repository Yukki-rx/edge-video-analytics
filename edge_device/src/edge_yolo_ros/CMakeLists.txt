cmake_minimum_required(VERSION 3.0.2)
project(edge_yolo_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/yolo_detector.py
  scripts/metrics_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
