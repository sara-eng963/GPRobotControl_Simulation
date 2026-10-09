# External-memory validation on the CAN/AVATAR branch.

add_executable(file_validated_storage_test
    Simulation/Tests/test_file_validated_storage.c
    Simulation/Storage/file_validated_storage.c
)

add_executable(validated_stream_buffer_test
    Simulation/Tests/test_validated_stream_buffer.c
    Simulation/Storage/validated_stream_buffer.c
)

add_executable(external_streaming_test
    Simulation/Tests/test_external_streaming.c
    Simulation/Storage/file_validated_storage.c
    Simulation/Storage/validated_stream_buffer.c
)

find_package(Threads REQUIRED)
foreach(test_name IN ITEMS trajectory_prefetch_test trajectory_prefetch_concurrent_test)
    if(test_name STREQUAL "trajectory_prefetch_test")
        set(prefetch_test_source Simulation/Tests/test_trajectory_prefetch.c)
    else()
        set(prefetch_test_source Simulation/Tests/test_trajectory_prefetch_concurrent.c)
    endif()
    add_executable(${test_name} ${prefetch_test_source}
        Simulation/Storage/trajectory_prefetch.c)
    target_compile_definitions(${test_name} PRIVATE TRAJECTORY_PREFETCH_HOST_TEST)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_link_libraries(${test_name} PRIVATE Threads::Threads)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 30)
endforeach()

add_executable(path_execution_prefetch_test
    StateMachine/Tests/test_path_execution_prefetch.c
    StateMachine/States/state_path_execution.c
    Simulation/Storage/trajectory_prefetch.c)
target_compile_options(path_execution_prefetch_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME path_execution_prefetch_test COMMAND path_execution_prefetch_test)
set_tests_properties(path_execution_prefetch_test PROPERTIES TIMEOUT 15)

add_executable(w25q512jv_verified_test
    Simulation/Tests/test_w25q512jv_verified.c
    Simulation/Renode/Storage/w25q512jv_flash.c
    Simulation/Renode/Storage/qspi_nor_validated_storage.c)
target_compile_options(w25q512jv_verified_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME w25q512jv_verified_test COMMAND w25q512jv_verified_test)

add_executable(stm32h7_w25q_fifo_test
    Simulation/Tests/test_stm32h7_w25q_fifo.c
    Simulation/Renode/Storage/stm32h7_w25q_bus.c)
target_compile_definitions(stm32h7_w25q_fifo_test PRIVATE W25Q_TEST_MMIO)
target_compile_options(stm32h7_w25q_fifo_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME stm32h7_w25q_fifo_test COMMAND stm32h7_w25q_fifo_test)

# These tests create relative flash images/CSV files. Keep each in the build
# tree, never in the source directory or in another test's working directory.
foreach(test_name IN ITEMS file_validated_storage_test validated_stream_buffer_test external_streaming_test)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-output/${test_name}")
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-output/${test_name}"
        TIMEOUT 60)
endforeach()
