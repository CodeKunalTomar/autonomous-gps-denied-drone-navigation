cmake_minimum_required(VERSION 3.8)
project(drone_visual_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(slam_bridge src/slam_bridge.cpp)
ament_target_dependencies(slam_bridge rclcpp geometry_msgs nav_msgs px4_msgs)
add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher rclcpp sensor_msgs)

install(TARGETS slam_bridge camera_info_publisher DESTINATION lib/${PROJECT_NAME})
install(TARGETS slam_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
