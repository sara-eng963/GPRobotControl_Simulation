# Include once from the end of the project's top-level CMakeLists.txt.
# The active whole-pipeline transport is SIL Kit CAN1.

if(ENABLE_SILKIT)

    add_executable(
        supervisor_silkit_sim

        ${CMAKE_SOURCE_DIR}/Simulation/supervisor_silkit_main.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/host_guidance_demo.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_protocol.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_receiver.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_sensor.c
        ${CMAKE_SOURCE_DIR}/Simulation/Storage/trajectory_prefetch.c
        # Host-only flash model. No change to physical STM32 firmware path.
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/w25q_nor_model.c
        ${CMAKE_SOURCE_DIR}/Simulation/Renode/Storage/w25q512jv_flash.c
        ${CMAKE_SOURCE_DIR}/Simulation/Renode/Storage/qspi_nor_validated_storage.c
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
        ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_drive_commissioning_port.c
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
        -DGP_HOST_TIMING=1
        -DGP_ENABLE_LEGACY_SILKIT_GUIDANCE=1
        -DGP_HOST_GUIDANCE_MOTION_TEST=1
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
        hand_guiding
        intercore_ipc
        intercore_validation_storage
        host_timing_trace
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


    # 4F2A: pure-protocol and rigid-wrench host unit tests.
    add_executable(test_virtual_ft_protocol
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_virtual_ft_protocol.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_protocol.c)
    target_compile_features(test_virtual_ft_protocol PRIVATE c_std_11)
    target_link_libraries(test_virtual_ft_protocol PRIVATE m)
    add_executable(test_virtual_ft_sensor
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_virtual_ft_sensor.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_sensor.c)
    target_compile_features(test_virtual_ft_sensor PRIVATE c_std_11)
    target_link_libraries(test_virtual_ft_sensor PRIVATE hand_guiding control_core m)
    enable_testing()
    add_test(NAME virtual_ft_protocol COMMAND test_virtual_ft_protocol)
    add_test(NAME virtual_ft_sensor_frame COMMAND test_virtual_ft_sensor)

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
        add_test(
            NAME supervisor_silkit_flash_sequence
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_flash_sequence.py
                ${SILKIT_ROOT}/bin/sil-kit-registry
                $<TARGET_FILE:avatar_m_silkit_motor_bank>
                $<TARGET_FILE:supervisor_silkit_sim>
        )
        set_tests_properties(supervisor_silkit_flash_sequence PROPERTIES
            TIMEOUT 150 RUN_SERIAL TRUE)
        # Opt-in Stage 4E2B full SIL Kit observer integration, no
        # admittance-driven motor dispatch. Each run owns registry port 8500.
        foreach(fault IN ITEMS none stale_ft bad_feedback)
            add_test(
                NAME supervisor_silkit_guidance_dry_run_${fault}
                COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_guidance_dryrun.py
                    ${SILKIT_ROOT}/bin/sil-kit-registry
                    $<TARGET_FILE:avatar_m_silkit_motor_bank>
                    $<TARGET_FILE:supervisor_silkit_sim>
                    ${fault}
            )
            set_tests_properties(
                supervisor_silkit_guidance_dry_run_${fault}
                PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
        endforeach()

        # Stage 4E3: host-only simulated motor dispatch. The physical
        # STM32 target never compiles GP_HOST_GUIDANCE_MOTION_TEST.
        foreach(fault IN ITEMS none stale_ft bad_feedback)
            add_test(
                NAME supervisor_silkit_guidance_sim_motor_${fault}
                COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_guidance_motion.py
                    ${SILKIT_ROOT}/bin/sil-kit-registry
                    $<TARGET_FILE:avatar_m_silkit_motor_bank>
                    $<TARGET_FILE:supervisor_silkit_sim>
                    ${fault}
            )
            set_tests_properties(
                supervisor_silkit_guidance_sim_motor_${fault}
                PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE)
        endforeach()

        # F1: real virtual motor motion observed for several seconds.
        # Run with GP_4F1_OUTPUT_DIR to preserve measurements and plots.
        add_test(
            NAME supervisor_silkit_guidance_4f1_sustained
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_guidance_4f1_sustained.py
                ${SILKIT_ROOT}/bin/sil-kit-registry
                $<TARGET_FILE:avatar_m_silkit_motor_bank>
                $<TARGET_FILE:supervisor_silkit_sim>
        )
        set_tests_properties(
            supervisor_silkit_guidance_4f1_sustained
            PROPERTIES TIMEOUT 80 RUN_SERIAL TRUE)
        add_test(
            NAME guidance_4f1_analyzer_fixture
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_guidance_4f1_analyzer.py
        )
        set_tests_properties(guidance_4f1_analyzer_fixture PROPERTIES TIMEOUT 10)

        # 4F2A end-to-end simulated force input with no GUI dependency.
        add_test(
            NAME supervisor_silkit_virtual_ft_input
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_virtual_ft.py
                ${SILKIT_ROOT}/bin/sil-kit-registry
                $<TARGET_FILE:avatar_m_silkit_motor_bank>
                $<TARGET_FILE:supervisor_silkit_sim>
        )
        set_tests_properties(supervisor_silkit_virtual_ft_input PROPERTIES
            TIMEOUT 100 RUN_SERIAL TRUE)

        # Host-only erase-delay/cancellation and transport-failure tests.
        foreach(case_name IN ITEMS cancel program_fail)
            add_test(
                NAME supervisor_silkit_flash_${case_name}
                COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_flash_worker_faults.py
                    ${SILKIT_ROOT}/bin/sil-kit-registry
                    $<TARGET_FILE:avatar_m_silkit_motor_bank>
                    $<TARGET_FILE:supervisor_silkit_sim>
                    ${case_name}
            )
            set_tests_properties(supervisor_silkit_flash_${case_name}
                PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
        endforeach()


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

    add_executable(
        virtual_ft_gui
        ${CMAKE_SOURCE_DIR}/HMI/virtual_ft_gui_app.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_protocol.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_theme.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_protocol.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_client.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_controls.c
    )
    target_compile_features(virtual_ft_gui PRIVATE c_std_11)
    target_compile_definitions(virtual_ft_gui PRIVATE _POSIX_C_SOURCE=200809L)
    target_include_directories(virtual_ft_gui PRIVATE
        ${CMAKE_SOURCE_DIR}/HMI)
    target_link_libraries(virtual_ft_gui PRIVATE raylib m)

    # GUI's pure control mapping and real loopback UDP sender are testable
    # without a graphical display or simulated CAN network.
    add_executable(test_virtual_ft_controls
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_virtual_ft_controls.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_controls.c)
    target_link_libraries(test_virtual_ft_controls PRIVATE m)
    add_executable(test_virtual_ft_client
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_virtual_ft_client.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_client.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_protocol.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_controls.c)
    target_link_libraries(test_virtual_ft_client PRIVATE m)
    enable_testing()
    add_test(NAME virtual_ft_controls COMMAND test_virtual_ft_controls)
    add_test(NAME virtual_ft_client_udp COMMAND test_virtual_ft_client)
    set_tests_properties(virtual_ft_client_udp PROPERTIES RUN_SERIAL TRUE)

endif()
