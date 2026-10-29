# Build independently for focused Stage 4 tests, or from the repository root.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(HandGuiding C)
    enable_testing()
endif()

add_library(hand_guiding STATIC
    ft_sensor.c
    ft_serial_transport.c
    sensor_config.c
    admittance_control.c
    admittance_matlab_reference.c
    cartesian_joint_mapper.c
    wrench_frame.c
    hand_guiding_pipeline.c
    supervisor_dry_run_adapter.c
    guidance_command_gate.c
)
target_compile_features(hand_guiding PUBLIC c_std_11)
target_include_directories(hand_guiding PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(hand_guiding PRIVATE -Wall -Wextra -Wpedantic)
# Standalone tests build ONLY the ControlCore units used by this mapper.
# Root builds instead reuse the real repository control_core target.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_library(hand_guiding_control_core STATIC
        ../ControlCore/Kinematics/dh_transform.c
        ../ControlCore/Kinematics/control_jacobian.c
        ../ControlCore/Kinematics/control_fk.c
        ../ControlCore/Math/matrix6.c
        ../ControlCore/Analysis/singularity.c
        ../ControlCore/Config/robot_config.c
        ../ServoDrive/AvatarM/avatar_m_position.c
    )
    target_compile_features(hand_guiding_control_core PUBLIC c_std_11)
    target_compile_options(hand_guiding_control_core PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(hand_guiding PUBLIC hand_guiding_control_core m)
else()
    target_link_libraries(hand_guiding PUBLIC control_core avatar_drives m)
endif()

foreach(test_name IN ITEMS test_ft_sensor test_sensor_config test_wrench_processing test_admittance_control test_cartesian_joint_mapper test_wrench_frame test_hand_guiding_pipeline test_supervisor_dry_run_adapter test_guidance_command_gate test_ft_serial_transport test_p05_serial_guidance)
    add_executable(${test_name} Tests/${test_name}.c)
    target_link_libraries(${test_name} PRIVATE hand_guiding)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 10)
endforeach()

# Host-only integration: these classes do not exist in either STM32 image.
# Reuse ControlCore, existing CANopen, validation and intercore streaming.
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_executable(p05_virtual_system
        ../Simulation/P05/p05_virtual_system.c
        ../Simulation/P05/p05_production_supervisor.c
        ../Simulation/Renode/Storage/w25q512jv_flash.c
        ../Simulation/Renode/Storage/qspi_nor_validated_storage.c
        ../Simulation/Tests/w25q_nor_model.c)
    # The live chain uses the same production units as the P04 compiled-source
    # fixture, including its C2B coordinator. Never link the desktop stack here.
    set(P05_FW ../Firmware/STM32H745)
    set(P05_SM ${P05_FW}/CM7/Application/StateMachine)
    file(GLOB P05_STATE_SOURCES CONFIGURE_DEPENDS ${P05_SM}/States/*.c)
    add_library(p05_production_control STATIC
        ${P05_STATE_SOURCES} ${P05_SM}/state_machine.c ${P05_SM}/supervisor_io.c
        ${P05_FW}/CM7/Application/CANComm/CANopen/canopen_master.c
        ${P05_FW}/CM7/Application/CANComm/CANopen/canopen_heartbeat.c
        ${P05_FW}/CM7/Application/CANComm/CANopen/canopen_nmt.c
        ${P05_FW}/CM7/Application/CANComm/CANopen/canopen_sdo.c
        ../ServoDrive/JointDrive/canopen_joint_drive_port.c
        ../ServoDrive/JointDrive/canopen_drive_commissioning_port.c)
    target_compile_features(p05_production_control PRIVATE c_std_11)
    target_include_directories(p05_production_control PUBLIC
        ../Firmware/Tests/supervisor_alignment
        ${P05_SM} ${P05_SM}/States ../StateMachine/Tests
        ${P05_FW}/Common/Inc ${P05_FW}/CM7/Application/Intercore)
    target_link_libraries(p05_production_control PUBLIC control_core avatar_drives)
    target_compile_options(p05_production_control PRIVATE
        -Wall -Wextra -Werror -UNDEBUG -ffunction-sections -fdata-sections)
    target_link_options(p05_virtual_system PRIVATE -Wl,--gc-sections)
    target_compile_features(p05_virtual_system PRIVATE c_std_11)
    target_include_directories(p05_virtual_system PRIVATE
        ${P05_SM} ${P05_SM}/States)
    target_link_libraries(p05_virtual_system PRIVATE
        hand_guiding control_core avatar_drives p05_production_control
        intercore_validation_storage intercore_trajectory_storage m)
    target_compile_options(p05_virtual_system PRIVATE
        -Wall -Wextra -Wpedantic -Werror -UNDEBUG
        -ffunction-sections -fdata-sections)

    # Existing validation code emits a pre-C2x array-qualifier
    # pedantic warning. Keep strict checks on new P05 sources.
    # Scope this exception to the legacy source in this host target.
    set_source_files_properties(
        ../StateMachine/States/state_path_validation.c
        PROPERTIES COMPILE_OPTIONS "-Wno-error=pedantic")
    # The production fixture also includes the legacy reset-flag enum
    # whose highest bit exceeds ISO C's signed int enumerator range.
    set_source_files_properties(
        ../Simulation/P05/p05_production_supervisor.c
        ${P05_SM}/States/state_path_validation.c
        PROPERTIES COMPILE_OPTIONS "-Wno-error=pedantic")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/p05_ctest_evidence")
    add_test(NAME p05_virtual_end_to_end
        COMMAND p05_virtual_system "${CMAKE_CURRENT_BINARY_DIR}/p05_ctest_evidence")
    set_tests_properties(p05_virtual_end_to_end PROPERTIES TIMEOUT 180)
endif()

# Compile the actual CM7 DMA bridge with HAL stubs, exercising callbacks and
# task-side bounds. The firmware build retains the real STM32 HAL translation.
add_executable(test_p05_uart_dma_bridge
    ../Firmware/Tests/test_p05_uart_dma_bridge.c)
target_compile_features(test_p05_uart_dma_bridge PRIVATE c_std_11)
target_include_directories(test_p05_uart_dma_bridge PRIVATE
    ../Firmware/STM32H745/CM7/Core/Inc)
target_compile_options(test_p05_uart_dma_bridge PRIVATE
    -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(test_p05_uart_dma_bridge PRIVATE hand_guiding)
add_test(NAME test_p05_uart_dma_bridge COMMAND test_p05_uart_dma_bridge)
set_tests_properties(test_p05_uart_dma_bridge PROPERTIES TIMEOUT 10)
