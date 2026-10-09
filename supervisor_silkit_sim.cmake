# Include once from the end of the project's top-level CMakeLists.txt.
# The active whole-pipeline transport is SIL Kit CAN1.

if(ENABLE_SILKIT)

    add_executable(
        supervisor_silkit_sim

        ${CMAKE_SOURCE_DIR}/Simulation/supervisor_silkit_main.c
        ${CMAKE_SOURCE_DIR}/Simulation/Storage/trajectory_prefetch.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_task.c
        ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_task.c
        ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_io.c
        ${CMAKE_SOURCE_DIR}/StateMachine/state_machine.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_boot.c

        ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_master.c
        ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_nmt.c
        ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_sdo.c
        ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_heartbeat.c

        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_pdo.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_drive.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_position.c

        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_homing.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_idle.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_teaching.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_validation.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_approach.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_execution.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_joint_drive_port.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_paused.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_fault.c
        ${CMAKE_SOURCE_DIR}/StateMachine/States/state_emergency_stop.c

        ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402/cia402.c
    )

    target_compile_features(
        supervisor_silkit_sim
        PRIVATE
        c_std_11
    )

    target_compile_options(
        supervisor_silkit_sim
        PRIVATE
        -Wall
        -Wextra
        -Werror=implicit-function-declaration
        -Werror=incompatible-pointer-types
    )

    target_include_directories(
        supervisor_silkit_sim
        PRIVATE

        ${CMAKE_SOURCE_DIR}/StateMachine
        ${CMAKE_SOURCE_DIR}/StateMachine/States
        ${CMAKE_SOURCE_DIR}/HMI

        ${CMAKE_SOURCE_DIR}/CANComm
        ${CMAKE_SOURCE_DIR}/CANComm/CANopen
        ${CMAKE_SOURCE_DIR}/CANComm/SILKit

        ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM
    )

    target_link_libraries(
        supervisor_silkit_sim
        PRIVATE

        silkit_can_backend
        freertos_kernel
        freertos_config
        control_core
        pthread
        m
    )

    # C sources call a C ABI implemented by the C++ SIL Kit backend.
    set_target_properties(
        supervisor_silkit_sim
        PROPERTIES
        LINKER_LANGUAGE CXX
    )


    add_executable(
        avatar_m_silkit_motor_bank

        ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN/avatar_m_silkit_motor_bank.cpp
        ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN/avatar_m_silkit_node.cpp

        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_pdo.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_position.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402/cia402.c
    )

    target_compile_features(
        avatar_m_silkit_motor_bank
        PRIVATE
        cxx_std_17
    )

    target_include_directories(
        avatar_m_silkit_motor_bank
        PRIVATE

        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/CANComm
        ${CMAKE_SOURCE_DIR}/CANComm/CANopen
        ${CMAKE_SOURCE_DIR}/CANComm/SILKit
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM
        ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/Sim
        ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402
        ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN
    )

    target_link_libraries(
        avatar_m_silkit_motor_bank
        PRIVATE

        silkit_can_backend
        Threads::Threads
        m
    )



    if(TARGET raylib)

        add_executable(
            can_live_monitor

            ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN/can_live_monitor.cpp
        )

        target_compile_features(
            can_live_monitor
            PRIVATE
            cxx_std_17
        )

        target_include_directories(
            can_live_monitor
            PRIVATE

            ${CMAKE_SOURCE_DIR}
            ${CMAKE_SOURCE_DIR}/CANComm
            ${CMAKE_SOURCE_DIR}/CANComm/CANopen
            ${CMAKE_SOURCE_DIR}/CANComm/SILKit
        )

        target_link_libraries(
            can_live_monitor
            PRIVATE

            silkit_can_backend
            raylib
            Threads::Threads
            m
        )

    endif()


    find_package(
        Python3
        COMPONENTS Interpreter
        QUIET
    )

    if(Python3_Interpreter_FOUND)

        enable_testing()

        add_test(
            NAME supervisor_silkit_sequence
            COMMAND
                ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_sequence.py
                ${SILKIT_ROOT}/bin/sil-kit-registry
                $<TARGET_FILE:avatar_m_silkit_motor_bank>
                $<TARGET_FILE:supervisor_silkit_sim>
        )

        set_tests_properties(
            supervisor_silkit_sequence
            PROPERTIES
            TIMEOUT 120
            RUN_SERIAL TRUE
        )

    endif()

endif()


# Operational panel is transport-agnostic and can be built independently.
if(TARGET raylib)

    add_executable(
        supervisor_panel

        ${CMAKE_SOURCE_DIR}/hmi.c
        ${CMAKE_SOURCE_DIR}/HMI/supervisor_panel_app.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_protocol.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_theme.c
    )

    target_compile_definitions(
        supervisor_panel
        PRIVATE
        _POSIX_C_SOURCE=200809L
    )

    target_include_directories(
        supervisor_panel
        PRIVATE
        ${CMAKE_SOURCE_DIR}/HMI
    )

    target_link_libraries(
        supervisor_panel
        PRIVATE
        raylib
        m
    )

endif()
