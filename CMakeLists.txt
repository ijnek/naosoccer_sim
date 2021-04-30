cmake_minimum_required(VERSION 3.5)
project(naosoccer_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  external/sexpresso/
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nao_interfaces REQUIRED)

# Build naosoccer_sim
add_library(naosoccer_sim_lib SHARED
  src/naosoccer_sim.cpp
  src/connection.cpp
  src/socket.cpp
  src/socketaddress.cpp
  src/sexp_creator.cpp
  src/sexp_parser.cpp
  src/sim_to_nao.cpp
  src/nao_to_sim.cpp
  src/nao_joints_pid.cpp
  src/sim_joints.cpp
  src/complementary_filter.cpp
  external/sexpresso/sexpresso/sexpresso.cpp)

ament_target_dependencies(naosoccer_sim_lib
  rclcpp
  nao_interfaces)

add_executable(naosoccer_sim src/naosoccer_sim_node.cpp)

target_link_libraries(naosoccer_sim naosoccer_sim_lib)

# Install
install(TARGETS
  naosoccer_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  urdf/nao.urdf
  rviz/nao.rviz
  DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS naosoccer_sim_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
