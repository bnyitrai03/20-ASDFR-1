cmake_minimum_required(VERSION 3.8)
project(assignment1_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(image_brightness_node 
  src/image_brightness_node.cpp
)

ament_target_dependencies(image_brightness_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  image_brightness_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()