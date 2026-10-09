cmake_minimum_required(VERSION 3.28)
project(GPRobotControl_Simulation C)

option(ENABLE_SILKIT "Build SIL Kit CAN simulation support" OFF)

# Optional PC transport support. SIL Kit source files remain unchanged.
if(ENABLE_SILKIT)
    enable_language(CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(SILKIT_ROOT "" CACHE PATH "Path to SIL Kit installation root")
    if(NOT SILKIT_ROOT)
        message(FATAL_ERROR "ENABLE_SILKIT=ON requires SILKIT_ROOT")
    endif()

    find_package(SilKit REQUIRED CONFIG PATHS "${SILKIT_ROOT}" NO_DEFAULT_PATH)
    find_package(Threads REQUIRED)

    add_library(silkit_can_backend CANComm/SILKit/silkit_can_backend.cpp)
    target_include_directories(silkit_can_backend PUBLIC CANComm CANComm/SILKit)
    target_link_libraries(silkit_can_backend PUBLIC SilKit::SilKit Threads::Threads)
    target_compile_features(silkit_can_backend PRIVATE cxx_std_17)
endif()

set(CMAKE_C_STANDARD 11)
enable_testing()

# FreeRTOS runs with its existing POSIX host port.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)
set(FREERTOS_PORT GCC_POSIX CACHE STRING "")
set(FREERTOS_HEAP "4" CACHE STRING "")
add_subdirectory(../FreeRTOS-Kernel FreeRTOS-Kernel)

# Unmodified robot algorithms and kinematics.
add_subdirectory(ControlCore)

# The only definitions of the AVATAR drive and CiA-402 implementation sources.
# Restrict the glob to the two existing source directories (no Sim/ or Tests/).
file(GLOB AVATAR_DRIVE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/ServoDrive/AvatarM/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ServoDrive/CiA402/*.c"
)
add_library(avatar_drives STATIC ${AVATAR_DRIVE_SOURCES})
target_compile_features(avatar_drives PUBLIC c_std_11)
target_include_directories(avatar_drives PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/ServoDrive/AvatarM
    ${CMAKE_CURRENT_SOURCE_DIR}/ServoDrive/CiA402
    ${CMAKE_CURRENT_SOURCE_DIR}/CANComm
    ${CMAKE_CURRENT_SOURCE_DIR}/CANComm/CANopen
)
target_link_libraries(avatar_drives PUBLIC m)

file(GLOB CANOPEN_STACK_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/CANComm/CANopen/*.c"
)
add_library(canopen_stack STATIC ${CANOPEN_STACK_SOURCES})
target_compile_features(canopen_stack PUBLIC c_std_11)
target_include_directories(canopen_stack PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/CANComm
    ${CMAKE_CURRENT_SOURCE_DIR}/CANComm/CANopen
)
target_link_libraries(canopen_stack PUBLIC avatar_drives)

# Raylib-based user interfaces, unchanged source/dependency lists.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/raylib ${CMAKE_BINARY_DIR}/raylib)

add_executable(robot_hmi
    hmi.c
    HMI/hmi_app.c
    HMI/hmi_protocol.c
    HMI/hmi_theme.c
)
target_compile_features(robot_hmi PRIVATE c_std_11)
target_include_directories(robot_hmi PRIVATE HMI)
target_link_libraries(robot_hmi PRIVATE raylib m)

add_executable(teaching_sim
    teaching_sim.c
    HMI/teaching_sim_app.c
    HMI/hmi_protocol.c
    HMI/hmi_theme.c
)
target_compile_features(teaching_sim PRIVATE c_std_11)
target_include_directories(teaching_sim PRIVATE HMI)
target_link_libraries(teaching_sim PRIVATE raylib m)

# Tests reuse the drive libraries: protocol implementations are no longer
# recompiled separately into each unit-test executable.
add_executable(path_validation_test
    StateMachine/Tests/test_path_validation.c
    StateMachine/States/state_path_validation.c
)
target_compile_features(path_validation_test PRIVATE c_std_11)
target_include_directories(path_validation_test PRIVATE
    StateMachine
    StateMachine/States
)
target_link_libraries(path_validation_test PRIVATE control_core avatar_drives m)
add_test(NAME path_validation_test COMMAND path_validation_test)

add_executable(runtime_states_test
    StateMachine/Tests/test_runtime_states.c
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c
)
target_compile_features(runtime_states_test PRIVATE c_std_11)
target_include_directories(runtime_states_test PRIVATE
    StateMachine
    StateMachine/States
)
target_link_libraries(runtime_states_test PRIVATE control_core m)
add_test(NAME runtime_states_test COMMAND runtime_states_test)

add_executable(state_machine_test
    StateMachine/Tests/test_state_machine.c
    StateMachine/state_machine.c
)
target_compile_features(state_machine_test PRIVATE c_std_11)
target_include_directories(state_machine_test PRIVATE
    StateMachine
    StateMachine/States
)
target_link_libraries(state_machine_test PRIVATE control_core m)
add_test(NAME state_machine_test COMMAND state_machine_test)

# The object target catches supervisor type/linkage errors in host builds;
# SIL Kit's dedicated simulator source list is intentionally not rewritten.
add_library(supervisor_task_compile_check OBJECT
    StateMachine/supervisor_task.c
    StateMachine/supervisor_io.c
    StateMachine/States/state_path_execution.c
    ServoDrive/JointDrive/canopen_joint_drive_port.c
    ServoDrive/JointDrive/canopen_drive_commissioning_port.c
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c
)
target_compile_features(supervisor_task_compile_check PRIVATE c_std_11)
target_compile_options(supervisor_task_compile_check PRIVATE
    -Werror=incompatible-pointer-types
)
target_include_directories(supervisor_task_compile_check PRIVATE
    StateMachine
    StateMachine/States
    CANComm
    CANComm/CANopen
    ServoDrive/CiA402
    ServoDrive/AvatarM
)
target_link_libraries(supervisor_task_compile_check PRIVATE
    freertos_kernel
    freertos_config
    control_core
    canopen_stack
)

# The simulator retains its existing SIL Kit, HMI, and virtual-drive wiring.
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_interface_test.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_silkit_sim.cmake")

add_executable(avatar_m_canopen_test
    CANComm/Tests/test_avatar_m_canopen.c
)
target_compile_features(avatar_m_canopen_test PRIVATE c_std_11)
target_link_libraries(avatar_m_canopen_test PRIVATE avatar_drives)
add_test(NAME avatar_m_canopen_test COMMAND avatar_m_canopen_test)

add_executable(canopen_master_test
    CANComm/Tests/test_canopen_master.c
    ServoDrive/JointDrive/canopen_joint_drive_port.c
    ServoDrive/JointDrive/canopen_drive_commissioning_port.c
)
target_compile_features(canopen_master_test PRIVATE c_std_11)
target_link_libraries(canopen_master_test PRIVATE canopen_stack m)
add_test(NAME canopen_master_test COMMAND canopen_master_test)

add_executable(can_backend_test CANComm/Tests/test_can_backend.c)
target_compile_features(can_backend_test PRIVATE c_std_11)
target_include_directories(can_backend_test PRIVATE CANComm)
add_test(NAME can_backend_test COMMAND can_backend_test)

if(ENABLE_SILKIT)
    add_executable(silkit_can_backend_test
        CANComm/Tests/test_silkit_can_backend.cpp
    )
    target_compile_features(silkit_can_backend_test PRIVATE cxx_std_17)
    target_link_libraries(silkit_can_backend_test PRIVATE silkit_can_backend)

    add_executable(silkit_smoke_test
        CANComm/Tests/test_silkit_smoke.cpp
    )
    target_link_libraries(silkit_smoke_test PRIVATE
        SilKit::SilKit
        Threads::Threads
    )
endif()

# Retain all external-memory test targets as defined in their separate file.
include("${CMAKE_CURRENT_SOURCE_DIR}/external_memory_tests.cmake")

add_executable(joint_drive_idle_test
    StateMachine/Tests/test_joint_drive_idle.c
    StateMachine/States/state_idle.c
)
target_compile_features(joint_drive_idle_test PRIVATE c_std_11)
target_compile_options(joint_drive_idle_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(joint_drive_idle_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive
)
add_test(NAME joint_drive_idle_test COMMAND joint_drive_idle_test)

add_executable(joint_drive_homing_test
    StateMachine/Tests/test_joint_drive_homing.c
    StateMachine/States/state_homing.c
)
target_compile_features(joint_drive_homing_test PRIVATE c_std_11)
target_compile_options(joint_drive_homing_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(joint_drive_homing_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive
)
target_link_libraries(joint_drive_homing_test PRIVATE control_core avatar_drives m)
add_test(NAME joint_drive_homing_test COMMAND joint_drive_homing_test)

add_executable(joint_drive_approach_test
    StateMachine/Tests/test_joint_drive_approach.c
    StateMachine/States/state_approach.c
)
target_compile_features(joint_drive_approach_test PRIVATE c_std_11)
target_compile_options(joint_drive_approach_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(joint_drive_approach_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive
)
target_link_libraries(joint_drive_approach_test PRIVATE control_core avatar_drives m)
add_test(NAME joint_drive_approach_test COMMAND joint_drive_approach_test)

add_executable(joint_drive_teaching_test
    StateMachine/Tests/test_joint_drive_teaching.c
    StateMachine/States/state_teaching.c
)
target_compile_features(joint_drive_teaching_test PRIVATE c_std_11)
target_compile_options(joint_drive_teaching_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(joint_drive_teaching_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive
)
target_link_libraries(joint_drive_teaching_test PRIVATE control_core avatar_drives m)
add_test(NAME joint_drive_teaching_test COMMAND joint_drive_teaching_test)

add_executable(joint_drive_boot_test
    StateMachine/Tests/test_joint_drive_boot.c
    StateMachine/States/state_boot.c
)
target_compile_features(joint_drive_boot_test PRIVATE c_std_11)
target_compile_options(joint_drive_boot_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types
)
target_include_directories(joint_drive_boot_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive
)
target_link_libraries(joint_drive_boot_test PRIVATE avatar_drives m)
add_test(NAME joint_drive_boot_test COMMAND joint_drive_boot_test)
