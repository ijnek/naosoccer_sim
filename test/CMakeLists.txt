# Build test_nao_to_sim
ament_add_gtest_executable(test_nao_to_sim 
  test_nao_to_sim.cpp)

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

# Build test_sim_to_nao
ament_add_gtest(test_sim_to_nao 
  test_sim_to_nao.cpp)

target_link_libraries(test_sim_to_nao
    naosoccer_sim_lib
)
