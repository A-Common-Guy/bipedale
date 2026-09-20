cmake_minimum_required(VERSION 3.16)
project(sandro_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/sync_model.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_models test/test_description.py
    TIMEOUT 30
  )
endif()

ament_package()
