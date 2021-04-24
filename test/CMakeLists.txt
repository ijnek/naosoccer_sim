ament_add_gtest_executable(test_nao_to_sim 
  test_nao_to_sim.cpp)

ament_target_dependencies(test_nao_to_sim
    rclcpp
    sensor_msgs
)

target_link_libraries(test_nao_to_sim
    nao_to_sim_lib
)

ament_add_test(test_nao_to_sim
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_nao_to_sim_launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    ENV
        TEST_EXECUTABLE=$<TARGET_FILE:test_nao_to_sim>
)