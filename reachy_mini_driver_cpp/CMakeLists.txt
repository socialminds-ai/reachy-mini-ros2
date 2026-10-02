cmake_minimum_required(VERSION 3.16)
project(reachy_mini_driver_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# hardware-independent logic (bus interface, kinematics wrapper, recorded moves)
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/bus.cpp src/kinematics.cpp src/limits.cpp src/moves.cpp src/trajectories.cpp)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen nlohmann_json::nlohmann_json)

# the node
ament_auto_add_library(${PROJECT_NAME}_node SHARED src/driver_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_auto_add_executable(driver_node src/main.cpp)
target_link_libraries(driver_node ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_moves test/test_moves.cpp)
  target_link_libraries(test_moves ${PROJECT_NAME}_lib)
  target_compile_definitions(test_moves PRIVATE TEST_MOVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/moves")

  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits ${PROJECT_NAME}_lib)

  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver ${PROJECT_NAME}_node)
  target_compile_definitions(test_driver PRIVATE TEST_MOVES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/moves")
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
