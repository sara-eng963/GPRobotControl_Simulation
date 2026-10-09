cmake_minimum_required(VERSION 3.28)

project(GPRobotControl_Simulation C)

option(
    ENABLE_SILKIT
    "Build SIL Kit CAN simulation support"
    OFF
)

if(ENABLE_SILKIT)

    enable_language(CXX)

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    set(
        SILKIT_ROOT
        ""
        CACHE PATH
        "Path to SIL Kit installation root"
    )

    if(NOT SILKIT_ROOT)
        message(
            FATAL_ERROR
            "ENABLE_SILKIT=ON requires SILKIT_ROOT"
        )
    endif()

    find_package(
        SilKit
        REQUIRED
        CONFIG
        PATHS "${SILKIT_ROOT}"
        NO_DEFAULT_PATH
    )

    find_package(Threads REQUIRED)

    add_library(
        silkit_can_backend

        CANComm/SILKit/silkit_can_backend.cpp
    )

    target_include_directories(
        silkit_can_backend
        PUBLIC
        CANComm
        CANComm/SILKit
    )

    target_link_libraries(
        silkit_can_backend
        PUBLIC

        SilKit::SilKit
        Threads::Threads
    )

    target_compile_features(
        silkit_can_backend
        PRIVATE
        cxx_std_17
    )

endif()

set(CMAKE_C_STANDARD 11)

# ----------------------------
# FreeRTOS configuration
# ----------------------------

add_library(freertos_config INTERFACE)

target_include_directories(
    freertos_config
    SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(
    freertos_config
    INTERFACE
    projCOVERAGE_TEST=0
)

set(FREERTOS_PORT GCC_POSIX CACHE STRING "")
set(FREERTOS_HEAP "4" CACHE STRING "")

add_subdirectory(
    ../FreeRTOS-Kernel
    FreeRTOS-Kernel
)

# ============================================================================
# CONTROL CORE
# ============================================================================

add_subdirectory(ControlCore)

# ============================================================================
# CANOPEN / AVATAR SIL KIT SUPERVISOR SIMULATOR
# ============================================================================
#
# The active end-to-end PC simulator is supervisor_silkit_sim, defined in
# supervisor_silkit_sim.cmake near the end of this file. It routes the real
# Supervisor/StateMachine through the SIL Kit CAN backend to six AVATAR
# virtual nodes on CAN1.
#
# ============================================================================

# ============================================================================
# RAYLIB
# ============================================================================

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

add_subdirectory(
    external/raylib
    ${CMAKE_BINARY_DIR}/raylib
)

# ============================================================================
# REAL ROBOT HMI PANEL + TEACHING-SIM WINDOW
# ============================================================================

add_executable(
    robot_hmi

    hmi.c

    HMI/hmi_app.c
    HMI/hmi_protocol.c
    HMI/hmi_theme.c
)

target_compile_features(
    robot_hmi
    PRIVATE
    c_std_11
)

target_include_directories(
    robot_hmi
    PRIVATE
    HMI
)

target_link_libraries(
    robot_hmi
    PRIVATE
    raylib
    m
)

add_executable(
    teaching_sim

    teaching_sim.c

    HMI/teaching_sim_app.c
    HMI/hmi_protocol.c
    HMI/hmi_theme.c
)

target_compile_features(
    teaching_sim
    PRIVATE
    c_std_11
)

target_include_directories(
    teaching_sim
    PRIVATE
    HMI
)

target_link_libraries(
    teaching_sim
    PRIVATE
    raylib
    m
)

# ============================================================================
# PATH VALIDATION STATE INTEGRATION TEST - AVATAR EXECUTION ARTIFACT
# ============================================================================

add_executable(
    path_validation_test

    StateMachine/Tests/test_path_validation.c
    StateMachine/States/state_path_validation.c

    ServoDrive/AvatarM/avatar_m_position.c
)

target_compile_features(
    path_validation_test
    PRIVATE
    c_std_11
)

target_include_directories(
    path_validation_test
    PRIVATE

    StateMachine
    StateMachine/States

    ServoDrive/AvatarM
)

target_link_libraries(
    path_validation_test
    PRIVATE

    control_core
    m
)

# ============================================================================
# GLOBAL STATE MACHINE UNIT TEST
# ============================================================================

add_executable(
    runtime_states_test
    StateMachine/Tests/test_runtime_states.c
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c
)

target_include_directories(
    runtime_states_test
    PRIVATE
    StateMachine
    StateMachine/States
)

target_link_libraries(
    runtime_states_test
    PRIVATE
    control_core
    m
)

add_executable(
    state_machine_test

    StateMachine/Tests/test_state_machine.c
    StateMachine/state_machine.c
)

target_include_directories(
    state_machine_test
    PRIVATE

    StateMachine
    StateMachine/States
)

target_link_libraries(
    state_machine_test
    PRIVATE

    control_core
    m
)

# ============================================================================
# SUPERVISOR RTOS TASK COMPILE CHECK
# ============================================================================

add_library(
    supervisor_task_compile_check
    OBJECT

    StateMachine/supervisor_task.c
    StateMachine/supervisor_io.c
    StateMachine/States/state_path_execution.c
    ServoDrive/JointDrive/canopen_joint_drive_port.c
    ServoDrive/JointDrive/canopen_drive_commissioning_port.c
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c
)

target_compile_features(
    supervisor_task_compile_check
    PRIVATE
    c_std_11
)

target_compile_options(
    supervisor_task_compile_check PRIVATE
    -Werror=incompatible-pointer-types
)

target_include_directories(
    supervisor_task_compile_check
    PRIVATE

    StateMachine
    StateMachine/States

    CANComm
    CANComm/CANopen

    ServoDrive/CiA402
    ServoDrive/AvatarM
)

target_link_libraries(
    supervisor_task_compile_check
    PRIVATE

    freertos_kernel
    freertos_config
    control_core
)
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_interface_test.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_silkit_sim.cmake")

# ============================================================================
# AVATAR M CANOPEN PROTOCOL TEST
# ============================================================================

add_executable(
    avatar_m_canopen_test

    CANComm/Tests/test_avatar_m_canopen.c
    ServoDrive/AvatarM/avatar_m_pdo.c
)

target_compile_features(
    avatar_m_canopen_test
    PRIVATE
    c_std_11
)

target_include_directories(
    avatar_m_canopen_test
    PRIVATE
    CANComm
    CANComm/CANopen
    ServoDrive/AvatarM
)

# ============================================================================
# SIX-AXIS CANOPEN MASTER INTEGRATION TEST
# ============================================================================

add_executable(
    canopen_master_test

    CANComm/Tests/test_canopen_master.c

    CANComm/CANopen/canopen_master.c
    ServoDrive/JointDrive/canopen_joint_drive_port.c
    ServoDrive/JointDrive/canopen_drive_commissioning_port.c
    CANComm/CANopen/canopen_nmt.c
    CANComm/CANopen/canopen_sdo.c
    CANComm/CANopen/canopen_heartbeat.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/AvatarM/avatar_m_pdo.c
    ServoDrive/AvatarM/avatar_m_drive.c
    ServoDrive/AvatarM/avatar_m_position.c
)

target_compile_features(
    canopen_master_test
    PRIVATE
    c_std_11
)

target_include_directories(
    canopen_master_test
    PRIVATE

    CANComm
    CANComm/CANopen

    ServoDrive/CiA402
    ServoDrive/AvatarM
)

target_link_libraries(
    canopen_master_test
    PRIVATE
    m
)


add_executable(
    can_backend_test
    CANComm/Tests/test_can_backend.c
)

target_compile_features(
    can_backend_test
    PRIVATE
    c_std_11
)

target_include_directories(
    can_backend_test
    PRIVATE
    CANComm
)

if(ENABLE_SILKIT)
add_executable(
    silkit_can_backend_test

    CANComm/Tests/test_silkit_can_backend.cpp
)

target_link_libraries(
    silkit_can_backend_test
    PRIVATE

    silkit_can_backend
)

target_compile_features(
    silkit_can_backend_test
    PRIVATE
    cxx_std_17
)

    add_executable(
        silkit_smoke_test

        CANComm/Tests/test_silkit_smoke.cpp
    )

    target_link_libraries(
        silkit_smoke_test
        PRIVATE

        SilKit::SilKit
        Threads::Threads
    )

endif()
include("${CMAKE_CURRENT_SOURCE_DIR}/external_memory_tests.cmake")

# The IDLE state is tested using a fake drive port, without CANopen or SIL Kit.
add_executable(joint_drive_idle_test
    StateMachine/Tests/test_joint_drive_idle.c
    StateMachine/States/state_idle.c
)
target_compile_features(joint_drive_idle_test PRIVATE c_std_11)
target_compile_options(joint_drive_idle_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration)
target_include_directories(joint_drive_idle_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive)
enable_testing()
add_test(NAME joint_drive_idle_test COMMAND joint_drive_idle_test)

# HOMING application-state regression using a fake six-axis drive service.
add_executable(joint_drive_homing_test
    StateMachine/Tests/test_joint_drive_homing.c
    StateMachine/States/state_homing.c
    ServoDrive/AvatarM/avatar_m_position.c
)
target_compile_features(joint_drive_homing_test PRIVATE c_std_11)
target_compile_options(joint_drive_homing_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration)
target_include_directories(joint_drive_homing_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive)
target_link_libraries(joint_drive_homing_test PRIVATE control_core m)
enable_testing()
add_test(NAME joint_drive_homing_test COMMAND joint_drive_homing_test)

# APPROACH regression without CANopen or SIL Kit: six-axis fake drive.
add_executable(joint_drive_approach_test
    StateMachine/Tests/test_joint_drive_approach.c
    StateMachine/States/state_approach.c
    ServoDrive/AvatarM/avatar_m_position.c
)
target_compile_features(joint_drive_approach_test PRIVATE c_std_11)
target_compile_options(joint_drive_approach_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types)
target_include_directories(joint_drive_approach_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive)
target_link_libraries(joint_drive_approach_test PRIVATE control_core m)
enable_testing()
add_test(NAME joint_drive_approach_test COMMAND joint_drive_approach_test)

# TEACHING capture regression with fake six-axis drive, no CANopen / SIL Kit.
add_executable(joint_drive_teaching_test
    StateMachine/Tests/test_joint_drive_teaching.c
    StateMachine/States/state_teaching.c
    ServoDrive/AvatarM/avatar_m_position.c
)
target_compile_features(joint_drive_teaching_test PRIVATE c_std_11)
target_compile_options(joint_drive_teaching_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types)
target_include_directories(joint_drive_teaching_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive)
target_link_libraries(joint_drive_teaching_test PRIVATE control_core m)
enable_testing()
add_test(NAME joint_drive_teaching_test COMMAND joint_drive_teaching_test)

# BOOT nonblocking state regression using fake motion/commissioning ports.
add_executable(joint_drive_boot_test
    StateMachine/Tests/test_joint_drive_boot.c
    StateMachine/States/state_boot.c
    ServoDrive/CiA402/cia402.c
)
target_compile_features(joint_drive_boot_test PRIVATE c_std_11)
target_compile_options(joint_drive_boot_test PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types)
target_include_directories(joint_drive_boot_test PRIVATE
    StateMachine StateMachine/States ServoDrive/JointDrive)
target_link_libraries(joint_drive_boot_test PRIVATE m)
enable_testing()
add_test(NAME joint_drive_boot_test COMMAND joint_drive_boot_test)
