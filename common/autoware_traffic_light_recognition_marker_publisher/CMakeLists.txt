cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_recognition_marker_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_recognition_marker_publisher.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light_recognition_marker_publisher::TrafficLightRecognitionMarkerPublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
