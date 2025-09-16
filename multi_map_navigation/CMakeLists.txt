cmake_minimum_required(VERSION 3.8)
project(multi_map_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(SQLite3 REQUIRED)  # <- added
find_package(std_srvs REQUIRED)
# Add executable
add_executable(multi_map_navigation src/multi_map_navigation.cpp)

ament_target_dependencies(multi_map_navigation
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
)

# Link SQLite3
target_link_libraries(multi_map_navigation SQLite::SQLite3)

# Install
install(TARGETS
  multi_map_navigation
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
