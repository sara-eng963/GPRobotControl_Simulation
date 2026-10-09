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

add_executable(qspi_storage_power_loss_test
    Simulation/Tests/test_qspi_storage_power_loss.c
    Simulation/Tests/w25q_nor_model.c
    Simulation/Renode/Storage/w25q512jv_flash.c
    Simulation/Renode/Storage/qspi_nor_validated_storage.c)
target_compile_options(qspi_storage_power_loss_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME qspi_storage_power_loss_test COMMAND qspi_storage_power_loss_test)
set_tests_properties(qspi_storage_power_loss_test PROPERTIES TIMEOUT 60)

add_executable(w25q512jv_protocol_test
    Simulation/Tests/test_w25q512jv_protocol.c
    Simulation/Tests/w25q_nor_model.c
    Simulation/Renode/Storage/w25q512jv_flash.c)
target_compile_options(w25q512jv_protocol_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME w25q512jv_protocol_test COMMAND w25q512jv_protocol_test)
set_tests_properties(w25q512jv_protocol_test PROPERTIES TIMEOUT 60)

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

# Batch 4: optional, genuine STM32H7/QUADSPI firmware runs in Renode.
# These remain CTest SKIPPED AT CONFIGURATION (not "passed") when tooling is
# missing; the build must remain usable without Renode or an ARM cross compiler.
option(ENABLE_RENODE_TESTS "Register Renode QSPI functional firmware tests when dependencies exist" ON)
if(ENABLE_RENODE_TESTS)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    find_program(RENODE_EXECUTABLE NAMES renode
        HINTS "$ENV{HOME}/renode_portable"
        DOC "Renode executable for STM32H7 QSPI functional tests")
    find_program(RENODE_ARM_GCC NAMES arm-none-eabi-gcc)
    find_program(RENODE_ARM_OBJCOPY NAMES arm-none-eabi-objcopy)
    find_program(RENODE_MAKE NAMES make)

    if(Python3_Interpreter_FOUND)
        add_test(NAME renode_qspi_runner_self_test
            COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_renode_qspi_tests.py" --self-test)
        set_tests_properties(renode_qspi_runner_self_test PROPERTIES
            LABELS "renode;external_memory" TIMEOUT 10)
    endif()

    if(Python3_Interpreter_FOUND AND RENODE_EXECUTABLE AND
       RENODE_ARM_GCC AND RENODE_ARM_OBJCOPY AND RENODE_MAKE)
        foreach(case IN ITEMS
            qspi_jedec_test
            qspi_rw_test
            qspi_pv_sample_test
            qspi_storage_backend_test
            qspi_streaming_test)
            add_test(NAME renode_${case}
                COMMAND "${Python3_EXECUTABLE}"
                    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_renode_qspi_tests.py"
                    --renode "${RENODE_EXECUTABLE}" --test "${case}" --timeout 150)
            set_tests_properties(renode_${case} PROPERTIES
                LABELS "renode;external_memory" TIMEOUT 210 RUN_SERIAL TRUE)
        endforeach()
        message(STATUS "Renode QSPI firmware tests REGISTERED: 5 cases + runner self-test")
    else()
        message(STATUS "Renode QSPI firmware tests NOT REGISTERED (missing optional Python3/Renode/arm-none-eabi-gcc/arm-none-eabi-objcopy/make)")
    endif()
else()
    message(STATUS "Renode QSPI firmware tests DISABLED by ENABLE_RENODE_TESTS=OFF")
endif()

# Batch 5: flash-backed prefetch under host 2 ms pacing.
# Functional integrity and fault detection only. WSL is not RTOS/hardware
# deadline evidence; CTest PASS does not certify 500 Hz determinism.
add_executable(flash_prefetch_500hz_test
    Simulation/Tests/test_flash_prefetch_500hz.c
    Simulation/Storage/trajectory_prefetch.c
    Simulation/Renode/Storage/qspi_nor_validated_storage.c
    Simulation/Renode/Storage/w25q512jv_flash.c
    Simulation/Tests/w25q_nor_model.c)
target_compile_options(flash_prefetch_500hz_test PRIVATE
    -O2 -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(flash_prefetch_500hz_test PRIVATE Threads::Threads)
add_test(NAME flash_prefetch_500hz_test COMMAND flash_prefetch_500hz_test)
set_tests_properties(flash_prefetch_500hz_test PROPERTIES
    TIMEOUT 60 RUN_SERIAL TRUE LABELS "external_memory;timing_model")
