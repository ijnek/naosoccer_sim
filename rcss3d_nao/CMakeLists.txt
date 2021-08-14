cmake_minimum_required(VERSION 3.8)
project(rcss3d_nao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcss3d_controller REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(rcss3d_controller_msgs REQUIRED)

# Build rcss3d_nao_lib library
add_library(rcss3d_nao_lib SHARED
  src/rcss3d_nao.cpp
  src/nao_to_sim.cpp
  src/sim_to_nao.cpp
  src/complementary_filter.cpp)

ament_target_dependencies(rcss3d_nao_lib
  rclcpp
  sensor_msgs
  nao_sensor_msgs
  nao_command_msgs
  rcss3d_controller_msgs)

# Build rcss3d_nao executable
add_executable(rcss3d_nao src/rcss3d_nao_node.cpp)
target_link_libraries(rcss3d_nao
  rcss3d_nao_lib
  rcss3d_controller::rcss3d_controller)

# Install
install(TARGETS
  rcss3d_nao
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS rcss3d_nao_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
