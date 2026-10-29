# Process/CLI contracts run without SIL Kit, Windows or a display.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
foreach(pair IN ITEMS
        "launcher_contracts|test_robot_launchers.py"
        "windows_matlab_contracts|test_windows_matlab.py"
        "verification_contracts|test_robot_verification.py"
        "installer_contracts|test_robot_installer.py")
    string(REPLACE "|" ";" parts "${pair}")
    list(GET parts 0 name)
    list(GET parts 1 script)
    add_test(NAME robot_${name} COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/${script})
    set_tests_properties(robot_${name} PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
endforeach()

# Live host composition; neither target links this code into ARM firmware.
if(ENABLE_SILKIT)
    # Separate host kernel configuration: legacy SIL Kit executables retain
    # their original allocation policy. Both use the existing POSIX port.
    get_target_property(ROBOT_KERNEL_ROOT freertos_kernel SOURCE_DIR)
    add_library(robot_freertos_config INTERFACE)
    target_link_libraries(robot_freertos_config INTERFACE freertos_config)
    target_compile_definitions(robot_freertos_config INTERFACE ROBOT_FREERTOS_STATIC=1)
    add_library(robot_freertos_kernel STATIC
        ${ROBOT_KERNEL_ROOT}/croutine.c ${ROBOT_KERNEL_ROOT}/event_groups.c
        ${ROBOT_KERNEL_ROOT}/list.c ${ROBOT_KERNEL_ROOT}/queue.c
        ${ROBOT_KERNEL_ROOT}/stream_buffer.c ${ROBOT_KERNEL_ROOT}/tasks.c
        ${ROBOT_KERNEL_ROOT}/timers.c ${ROBOT_KERNEL_ROOT}/portable/MemMang/heap_4.c
        ${CMAKE_SOURCE_DIR}/Simulation/Robot/host_static_memory.c)
    target_link_libraries(robot_freertos_kernel PUBLIC robot_freertos_config
        freertos_kernel_include freertos_kernel_port)
    set(ROBOT_FW ${CMAKE_SOURCE_DIR}/Firmware/STM32H745)
    set(ROBOT_SM ${ROBOT_FW}/CM7/Application/StateMachine)
    file(GLOB ROBOT_STATES CONFIGURE_DEPENDS ${ROBOT_SM}/States/*.c)
    add_executable(robot_simulation
        ${CMAKE_SOURCE_DIR}/Simulation/Robot/robot_runtime.c
        ${ROBOT_SM}/supervisor_task.c ${ROBOT_SM}/supervisor_io.c
        ${ROBOT_SM}/state_machine.c ${ROBOT_STATES}
        ${ROBOT_FW}/CM7/Application/CANComm/CANopen/canopen_master.c
        ${ROBOT_FW}/CM7/Application/CANComm/CANopen/canopen_heartbeat.c
        ${ROBOT_FW}/CM7/Application/CANComm/CANopen/canopen_nmt.c
        ${ROBOT_FW}/CM7/Application/CANComm/CANopen/canopen_sdo.c
        ${ROBOT_FW}/Common/Src/ipc_mailbox_freertos.c
        ${ROBOT_FW}/Common/Src/trajectory_stream.c
        ${ROBOT_FW}/CM7/Application/Intercore/validation_storage_bridge.c
        ${ROBOT_FW}/CM7/Application/Intercore/trajectory_storage_service.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_joint_drive_port.c
        ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_drive_commissioning_port.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_task.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/host_guidance_demo.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_protocol.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_receiver.c
        ${CMAKE_SOURCE_DIR}/Simulation/Guidance/virtual_ft_sensor.c
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/w25q_nor_model.c
        ${CMAKE_SOURCE_DIR}/Simulation/Renode/Storage/w25q512jv_flash.c
        ${CMAKE_SOURCE_DIR}/Simulation/Renode/Storage/qspi_nor_validated_storage.c)
    target_include_directories(robot_simulation PRIVATE
        ${ROBOT_SM} ${ROBOT_SM}/States ${ROBOT_FW}/Common/Inc
        ${ROBOT_FW}/CM7/Application/Intercore)
    target_compile_definitions(robot_simulation PRIVATE
        ROBOT_PRODUCTION_SIMULATION=1 IPC_FREERTOS_BACKEND=1 IPC_POSIX_TEST=1
        HAND_GUIDING_ENABLE_LEGACY_SILKIT=1 HOST_TIMING_TRACE_ENABLED=1)
    target_compile_options(robot_simulation PRIVATE -Wall -Wextra
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -include ${ROBOT_FW}/CM7/Application/CANComm/CANopen/canopen_master.h)
    target_link_libraries(robot_simulation PRIVATE silkit_can_backend robot_freertos_kernel
        robot_freertos_config hand_guiding control_core avatar_drives host_timing_trace pthread m)
    set_target_properties(robot_simulation PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME robot_production_silkit_sequence COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_robot_simulation.py
        ${SILKIT_ROOT}/bin/sil-kit-registry $<TARGET_FILE:avatar_m_silkit_motor_bank>
        $<TARGET_FILE:robot_simulation>)
    set_tests_properties(robot_production_silkit_sequence PROPERTIES TIMEOUT 150 RUN_SERIAL TRUE)
    add_test(NAME robot_production_virtual_ft COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_silkit_virtual_ft.py
        ${SILKIT_ROOT}/bin/sil-kit-registry $<TARGET_FILE:avatar_m_silkit_motor_bank>
        $<TARGET_FILE:robot_simulation>)
    set_tests_properties(robot_production_virtual_ft PROPERTIES TIMEOUT 100 RUN_SERIAL TRUE)
endif()
