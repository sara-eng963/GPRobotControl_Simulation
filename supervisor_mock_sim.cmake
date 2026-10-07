# Include once from the end of the project's top-level CMakeLists.txt.
# The real SOEM backend and old simulator main MUST NOT be linked into this target.
add_executable(supervisor_mock_sim
    ${CMAKE_SOURCE_DIR}/Simulation/supervisor_mock_main.c
    ${CMAKE_SOURCE_DIR}/Simulation/MockHardware/mock_backend.c
    ${CMAKE_SOURCE_DIR}/HMI/hmi_task.c
    ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_task.c
    ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_io.c
    ${CMAKE_SOURCE_DIR}/StateMachine/state_machine.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_boot.c

    ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN_GUI/sim_can_bus.c
    ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_master.c
    ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_nmt.c
    ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_sdo.c
    ${CMAKE_SOURCE_DIR}/CANComm/CANopen/canopen_heartbeat.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_pdo.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_drive.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/avatar_m_position.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c

    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_homing.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_idle.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_teaching.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_validation.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_approach.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_execution.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_paused.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_fault.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_emergency_stop.c
    ${CMAKE_SOURCE_DIR}/EtherCATComm/ethercat_master.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402/cia402.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC/a6ec_pdo.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC/a6ec_drive.c)
target_compile_features(supervisor_mock_sim PRIVATE c_std_11)
target_compile_options(supervisor_mock_sim PRIVATE -Wall -Wextra -Werror=implicit-function-declaration)
target_include_directories(supervisor_mock_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/StateMachine ${CMAKE_SOURCE_DIR}/HMI
    ${CMAKE_SOURCE_DIR}/EtherCATComm ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC
    ${CMAKE_SOURCE_DIR}/CANComm ${CMAKE_SOURCE_DIR}/CANComm/CANopen
    ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM/Sim
    ${CMAKE_SOURCE_DIR}/Simulation/AvatarM_CAN_GUI)
target_link_libraries(supervisor_mock_sim PRIVATE
    freertos_kernel freertos_config control_core soem pthread m)

# Optional panel derived from the current project GUI; original panel unchanged.
if(TARGET raylib)
    add_executable(supervisor_panel
        ${CMAKE_SOURCE_DIR}/hmi.c
        ${CMAKE_SOURCE_DIR}/HMI/supervisor_panel_app.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_protocol.c
        ${CMAKE_SOURCE_DIR}/HMI/hmi_theme.c)
    target_compile_definitions(supervisor_panel PRIVATE _POSIX_C_SOURCE=200809L)
    target_include_directories(supervisor_panel PRIVATE ${CMAKE_SOURCE_DIR}/HMI)
    target_link_libraries(supervisor_panel PRIVATE raylib m)
endif()
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    enable_testing()
    add_test(NAME supervisor_mock_sequence
        COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/Simulation/Tests/test_supervisor_mock_sequence.py
        $<TARGET_FILE:supervisor_mock_sim>)
    set_tests_properties(supervisor_mock_sequence PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
endif()
