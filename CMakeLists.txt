cmake_minimum_required(VERSION 3.5)
project(naosoccer_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nao_interfaces REQUIRED)
find_package(rcss3d_controller_msgs REQUIRED)

add_executable(sim src/sim.cpp)

ament_target_dependencies(
  sim
  rclcpp
  nao_interfaces
  rcss3d_controller_msgs)

install(TARGETS
  sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(FILES
  urdf/nao.urdf
  rviz/nao.rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
