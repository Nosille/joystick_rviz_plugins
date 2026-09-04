cmake_minimum_required(VERSION 3.20)
project(joystick_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(PLUGIN_LIBRARY joystick_aggregrator_controller)

add_library(${PLUGIN_LIBRARY} SHARED
  include/joystick_rviz_plugins/joystick_aggregator_panel.hpp
  src/joystick_aggregator_panel.cpp
)

target_include_directories(${PLUGIN_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PLUGIN_LIBRARY}
  rclcpp::rclcpp
  pluginlib::pluginlib
  rviz_common::rviz_common
  ${sensor_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PLUGIN_LIBRARY}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

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
