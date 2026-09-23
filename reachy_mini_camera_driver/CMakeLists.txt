cmake_minimum_required(VERSION 3.16)
project(reachy_mini_camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

ament_auto_add_library(${PROJECT_NAME}_node SHARED src/camera_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PkgConfig::GST)
ament_auto_add_executable(camera_node src/main.cpp)
target_link_libraries(camera_node ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
