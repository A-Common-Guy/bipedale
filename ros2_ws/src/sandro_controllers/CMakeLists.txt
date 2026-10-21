cmake_minimum_required(VERSION 3.16)
project(sandro_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(sandro_controllers SHARED
  src/control_core.cpp
  src/driving_balancer_controller.cpp
  src/kinematics.cpp
  src/lqr.cpp
  src/lqr_balancer_controller.cpp
  src/lqr_core.cpp
  src/rl_policy_controller.cpp
  src/rl_policy_core.cpp
  src/timed_effort_controller.cpp
  src/whole_body_controller.cpp
  src/whole_body_core.cpp
  src/whole_body_model.cpp
  src/whole_body_parameters.cpp
)
target_include_directories(sandro_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sandro_controllers
  ament_index_cpp controller_interface diagnostic_msgs geometry_msgs hardware_interface
  pluginlib rclcpp rcl_interfaces realtime_tools std_msgs std_srvs)
target_link_libraries(sandro_controllers Eigen3::Eigen)

pluginlib_export_plugin_description_file(
  controller_interface sandro_controllers_plugins.xml)

install(TARGETS sandro_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY policies DESTINATION share/${PROJECT_NAME})
install(FILES README.md sandro_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sandro_control
    test/test_control.cpp
    src/control_core.cpp
    src/kinematics.cpp
    src/lqr_core.cpp)
  target_include_directories(test_sandro_control PRIVATE include)
  ament_target_dependencies(test_sandro_control rclcpp)

  ament_add_gtest(test_sandro_whole_body
    test/test_whole_body.cpp
    src/kinematics.cpp
    src/lqr.cpp
    src/whole_body_core.cpp
    src/whole_body_model.cpp)
  target_include_directories(test_sandro_whole_body PRIVATE include)
  target_link_libraries(test_sandro_whole_body Eigen3::Eigen)

  ament_add_gtest(test_sandro_rl_policy
    test/test_rl_policy.cpp
    src/kinematics.cpp
    src/rl_policy_core.cpp)
  target_include_directories(test_sandro_rl_policy PRIVATE include)
  target_link_libraries(test_sandro_rl_policy Eigen3::Eigen)
  target_compile_definitions(test_sandro_rl_policy PRIVATE
    SANDRO_CONTROLLERS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_export_include_directories(include)
ament_export_dependencies(controller_interface Eigen3)
ament_export_libraries(sandro_controllers)
ament_package()
