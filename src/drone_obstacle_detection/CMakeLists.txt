cmake_minimum_required(VERSION 3.8)
project(drone_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(obstacle_detector src/obstacle_detector.cpp)
add_executable(obstacle_detector_debug src/obstacle_detector_debug.cpp)
ament_target_dependencies(obstacle_detector_debug rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

install(TARGETS obstacle_detector obstacle_detector_debug DESTINATION lib/${PROJECT_NAME})

# Specify dependencies for the executable
ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  px4_msgs
  cv_bridge
  OpenCV
)

# Install the executable
install(TARGETS obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
