cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(armor_detector_node
  src/main.cpp
  src/armor_detector_node.cpp
  src/armor_finder.cpp
  src/pnp_solver.cpp
  src/visualizer.cpp
)

ament_target_dependencies(armor_detector_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
)

install(TARGETS armor_detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
