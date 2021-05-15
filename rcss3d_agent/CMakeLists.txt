cmake_minimum_required(VERSION 3.5)
project(rcss3d_agent)

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
find_package(geometry_msgs REQUIRED)

# Build rcss3d_agent
add_library(rcss3d_agent_lib SHARED
  src/rcss3d_agent.cpp
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

ament_target_dependencies(rcss3d_agent_lib
  rclcpp
  nao_interfaces
  geometry_msgs)

add_executable(rcss3d_agent src/rcss3d_agent_node.cpp)

target_link_libraries(rcss3d_agent rcss3d_agent_lib)

# Install
install(TARGETS
  rcss3d_agent
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/)

install(
  TARGETS rcss3d_agent_lib
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
