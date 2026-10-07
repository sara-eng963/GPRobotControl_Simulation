# Include once from the end of the top-level CMakeLists.txt.
add_executable(supervisor_interface_test
    ${CMAKE_SOURCE_DIR}/StateMachine/Tests/test_supervisor_interfaces.c
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
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_paused.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_fault.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_emergency_stop.c
    ${CMAKE_SOURCE_DIR}/EtherCATComm/ethercat_master.c
    ${CMAKE_SOURCE_DIR}/EtherCATComm/SOEM/soem_backend.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402/cia402.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC/a6ec_pdo.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC/a6ec_drive.c
)
target_compile_features(supervisor_interface_test PRIVATE c_std_11)
target_compile_options(supervisor_interface_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration)
target_include_directories(supervisor_interface_test PRIVATE
    ${CMAKE_SOURCE_DIR}/StateMachine
    ${CMAKE_SOURCE_DIR}/StateMachine/States
    ${CMAKE_SOURCE_DIR}/EtherCATComm
    ${CMAKE_SOURCE_DIR}/EtherCATComm/SOEM
    ${CMAKE_SOURCE_DIR}/CANComm
    ${CMAKE_SOURCE_DIR}/CANComm/CANopen
    ${CMAKE_SOURCE_DIR}/ServoDrive/CiA402
    ${CMAKE_SOURCE_DIR}/ServoDrive/A6EC
    ${CMAKE_SOURCE_DIR}/ServoDrive/AvatarM)
target_link_libraries(supervisor_interface_test PRIVATE
    freertos_kernel freertos_config control_core soem pthread m)
enable_testing()
add_test(NAME supervisor_interface_test COMMAND supervisor_interface_test)
set_tests_properties(supervisor_interface_test PROPERTIES TIMEOUT 15)
