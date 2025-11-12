cmake_minimum_required(VERSION 3.8)
project(armor_detector)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(armor_detector_node src/armor_detector_node.cpp)
ament_target_dependencies(armor_detector_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV
)
install(TARGETS armor_detector_node DESTINATION lib/${PROJECT_NAME})