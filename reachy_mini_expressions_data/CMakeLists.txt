cmake_minimum_required(VERSION 3.16)
project(reachy_mini_expressions_data)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
ament_auto_package(INSTALL_TO_SHARE emotions dances sounds)
