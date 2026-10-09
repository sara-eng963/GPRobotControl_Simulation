# Supervisor interface integration test (CANopen protocol comes from
# canopen_stack / avatar_drives instead of per-executable source duplication).
add_executable(supervisor_interface_test
    ${CMAKE_SOURCE_DIR}/StateMachine/Tests/test_supervisor_interfaces.c
    ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_task.c
    ${CMAKE_SOURCE_DIR}/StateMachine/supervisor_io.c
    ${CMAKE_SOURCE_DIR}/StateMachine/state_machine.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_boot.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_homing.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_idle.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_teaching.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_validation.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_approach.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_path_execution.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_paused.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_fault.c
    ${CMAKE_SOURCE_DIR}/StateMachine/States/state_emergency_stop.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_joint_drive_port.c
    ${CMAKE_SOURCE_DIR}/ServoDrive/JointDrive/canopen_drive_commissioning_port.c
)
target_compile_features(supervisor_interface_test PRIVATE c_std_11)
target_compile_options(supervisor_interface_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(supervisor_interface_test PRIVATE
    ${CMAKE_SOURCE_DIR}/StateMachine
    ${CMAKE_SOURCE_DIR}/StateMachine/States
)
target_link_libraries(supervisor_interface_test PRIVATE
    freertos_kernel freertos_config control_core canopen_stack intercore_ipc pthread m
)
add_test(NAME supervisor_interface_test COMMAND supervisor_interface_test)
set_tests_properties(supervisor_interface_test PROPERTIES TIMEOUT 15)
