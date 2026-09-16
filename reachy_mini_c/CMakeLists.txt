cmake_minimum_required(VERSION 3.16)
project(reachy_mini_c VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- Build the Rust workspace with cargo ---------------------------------------
find_program(CARGO cargo REQUIRED)
set(RUST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rust)
set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/cargo)
set(RUST_LIB ${RUST_TARGET_DIR}/release/${CMAKE_SHARED_LIBRARY_PREFIX}reachy_mini_c${CMAKE_SHARED_LIBRARY_SUFFIX})

file(GLOB_RECURSE RUST_SOURCES ${RUST_DIR}/*.rs ${RUST_DIR}/*.toml ${RUST_DIR}/*.json)
add_custom_command(
  OUTPUT ${RUST_LIB}
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR}
          ${CARGO} build --release --manifest-path ${RUST_DIR}/Cargo.toml
  DEPENDS ${RUST_SOURCES}
  COMMENT "Building reachy_mini_c (Rust) with cargo"
  VERBATIM)
add_custom_target(reachy_mini_c_cargo ALL DEPENDS ${RUST_LIB})

# --- Expose it as an imported CMake target that downstream packages can link ---
add_library(reachy_mini_c SHARED IMPORTED GLOBAL)
add_dependencies(reachy_mini_c reachy_mini_c_cargo)
set_target_properties(reachy_mini_c PROPERTIES
  IMPORTED_LOCATION ${RUST_LIB}
  INTERFACE_INCLUDE_DIRECTORIES $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(FILES ${RUST_LIB} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Downstream: find_package(reachy_mini_c) then target_link_libraries(x reachy_mini_c::reachy_mini_c).
# The library is an imported (cargo-built) target, so it is exported through the
# CONFIG_EXTRAS file below rather than ament_export_libraries(); ament_auto's
# <pkg>_LIBRARIES / _INCLUDE_DIRS variables are set there too.
install(FILES cmake/reachy_mini_c-extras.cmake DESTINATION share/${PROJECT_NAME}/cmake)

# --- Tests ---------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics reachy_mini_c)
  add_dependencies(test_kinematics reachy_mini_c_cargo)
endif()

ament_auto_package(CONFIG_EXTRAS cmake/reachy_mini_c-extras.cmake)
