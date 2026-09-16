cmake_minimum_required(VERSION 3.16)
project(reachy_mini_driver_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(reachy_mini_c REQUIRED)
find_package(reachy_mini_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- hardware-independent logic (bus interface, kinematics wrapper, moves) ----
add_library(${PROJECT_NAME}_lib src/bus.cpp src/kinematics.cpp src/moves.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  reachy_mini_c::reachy_mini_c Eigen3::Eigen nlohmann_json::nlohmann_json)

# --- the node ----------------------------------------------------------------
add_library(${PROJECT_NAME}_node src/driver_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME}_lib
  rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${reachy_mini_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
add_executable(driver_node src/main.cpp)
target_link_libraries(driver_node ${PROJECT_NAME}_node)

install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- tests -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_moves test/test_moves.cpp)
  target_link_libraries(test_moves ${PROJECT_NAME}_lib)
  target_compile_definitions(test_moves PRIVATE TEST_MOVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/moves")

  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver ${PROJECT_NAME}_node)
  target_compile_definitions(test_driver PRIVATE TEST_MOVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/moves")
endif()

ament_package()
