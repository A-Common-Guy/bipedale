cmake_minimum_required(VERSION 3.16)
project(sandro_sim LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# One library for both plugins: LatencySystem (a hardware_interface system)
# and MitDrivePlugin (a mujoco_ros2_control plugin) share drive_bus(), which
# only works if they share the library that holds it.
add_library(sandro_sim SHARED
  src/drive_bus.cpp
  src/latency_system.cpp
  src/mit_drive_plugin.cpp
)
target_include_directories(sandro_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sandro_sim
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(sandro_sim
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins)

pluginlib_export_plugin_description_file(hardware_interface sandro_sim_plugins.xml)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins sandro_sim_mujoco_plugins.xml)

install(TARGETS sandro_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES sandro_sim_plugins.xml sandro_sim_mujoco_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_delay_line test/test_delay_line.cpp)
  target_include_directories(test_delay_line PRIVATE include)
  ament_add_gtest(test_mit_drive test/test_mit_drive.cpp)
  target_include_directories(test_mit_drive PRIVATE include)
  ament_add_gtest(test_drive_bus test/test_drive_bus.cpp)
  target_include_directories(test_drive_bus PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(sandro_sim)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
