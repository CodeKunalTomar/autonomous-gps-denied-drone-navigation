cmake_minimum_required(VERSION 3.8)
project(drone_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav2_common REQUIRED)

# Add executable
add_executable(path_planner src/path_planner.cpp)

# Specify dependencies for the executable
ament_target_dependencies(path_planner
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  nav2_common
)

# Install the executable
install(TARGETS path_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
