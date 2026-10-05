cmake_minimum_required(VERSION 3.28)

project(GPRobotControl_Simulation C)

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

# ----------------------------
# SOEM
# ----------------------------

add_subdirectory(
    ../SOEM
    SOEM-build
)

# ============================================================================
# CONTROL CORE
# ============================================================================

add_subdirectory(ControlCore)

# ============================================================================
# PC ROBOT SIMULATOR
# ============================================================================
#
# Simulation/simulator_main.c is the PC entry point. It runs the actual global
# StateMachine under the FreeRTOS POSIX port and connects it to KickCAT/SOEM,
# the desktop HMI, simulated hand-guiding support and MATLAB telemetry.
#
# Root main.c is intentionally NOT part of the PC build; it is reserved for
# the future real STM32/FreeRTOS robot application.
# ============================================================================

add_executable(
    robot_simulator

    Simulation/simulator_main.c

    HMI/hmi_state_bridge.c

    StateMachine/state_machine.c
    StateMachine/States/state_boot.c
    StateMachine/States/state_homing.c
    StateMachine/States/state_idle.c
    StateMachine/States/state_teaching.c
    StateMachine/States/state_path_validation.c
    StateMachine/States/state_approach.c
    StateMachine/States/state_path_execution.c
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c
    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c

    StateMachine/state_machine.c
    
)

target_compile_features(
    robot_simulator
    PRIVATE
    c_std_11
)

target_include_directories(
    robot_simulator
    PRIVATE

    HMI

    StateMachine
    StateMachine/States

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    robot_simulator
    PRIVATE

    freertos_kernel
    freertos_config

    control_core
    soem
    pthread
    m
)


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
# MODULAR ROBOT MOTION HMI
# ============================================================================

add_executable(
    mock_hmi

    hmi_mock.c

    HMI-Mock/hmi_app.c
    HMI-Mock/hmi_protocol.c
    HMI-Mock/hmi_theme.c
    HMI-Mock/hmi_runtime_guard.c
    HMI-Mock/waypoint_editor_3d.c
    HMI-Mock/workspace_monitor.c
)

target_compile_features(
    mock_hmi
    PRIVATE
    c_std_11
)

target_include_directories(
    mock_hmi
    PRIVATE
    HMI-Mock
)

target_link_libraries(
    mock_hmi
    PRIVATE
    raylib
    control_core
    m
)

# ---------------------------------------------------------------------------
# HMI runtime isolation for the PC EtherCAT simulation
# ---------------------------------------------------------------------------
#
# hmi_runtime_guard.c wraps these raylib calls so the GUI is capped at 20 FPS
# and 4x MSAA is removed.  This keeps WSLg rendering from competing as heavily
# with the 1 ms FreeRTOS/SOEM + KickCAT simulation.
#
# The controller timing is unchanged.

target_link_options(
    mock_hmi
    PRIVATE
    -Wl,--wrap=SetTargetFPS
    -Wl,--wrap=SetConfigFlags
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
# BOOT STATE TEST
# ============================================================================

add_executable(
    boot_test

    StateMachine/Tests/test_boot.c
    StateMachine/States/state_boot.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c
)

target_include_directories(
    boot_test
    PRIVATE

    StateMachine
    StateMachine/States

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    boot_test
    PRIVATE

    soem
    pthread
    m
)

# ============================================================================
# HOMING STATE TEST
# ============================================================================

add_executable(
    homing_test

    StateMachine/Tests/test_homing.c

    StateMachine/States/state_boot.c
    StateMachine/States/state_homing.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c
)

target_include_directories(
    homing_test
    PRIVATE

    StateMachine
    StateMachine/States

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    homing_test
    PRIVATE

    control_core
    soem
    pthread
    m
)

# ============================================================================
# IDLE STATE TEST
# ============================================================================

add_executable(
    idle_test

    StateMachine/Tests/test_idle.c

    StateMachine/States/state_boot.c
    StateMachine/States/state_homing.c
    StateMachine/States/state_idle.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c
)

target_include_directories(
    idle_test
    PRIVATE

    StateMachine
    StateMachine/States

    ControlCore/Config
    ControlCore/Math
    ControlCore/Trajectory

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    idle_test
    PRIVATE

    control_core
    soem
    pthread
    m
)

# ============================================================================
# TEACHING STATE INTEGRATION TEST
# ============================================================================

add_executable(
    teaching_test

    StateMachine/Tests/test_teaching.c

    StateMachine/States/state_boot.c
    StateMachine/States/state_homing.c
    StateMachine/States/state_idle.c
    StateMachine/States/state_teaching.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c
)

target_include_directories(
    teaching_test
    PRIVATE

    StateMachine
    StateMachine/States

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    teaching_test
    PRIVATE

    control_core
    soem
    pthread
    m
)

# ============================================================================
# PATH VALIDATION STATE INTEGRATION TEST
# ============================================================================
#
# test_path_validation.c has not been added to the repository yet.
# Keep the target available automatically once that test file exists, without
# preventing unrelated targets (such as state_machine_test) from configuring.
#
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/StateMachine/Tests/test_path_validation.c")

    add_executable(
        path_validation_test

        StateMachine/Tests/test_path_validation.c

        StateMachine/States/state_teaching.c
        StateMachine/States/state_path_validation.c

        EtherCATComm/ethercat_master.c
        EtherCATComm/SOEM/soem_backend.c

        ServoDrive/CiA402/cia402.c

        ServoDrive/A6EC/a6ec_pdo.c
        ServoDrive/A6EC/a6ec_drive.c
    )

    target_include_directories(
        path_validation_test
        PRIVATE

        StateMachine
        StateMachine/States

        EtherCATComm
        EtherCATComm/SOEM

        ServoDrive/CiA402
        ServoDrive/A6EC
    )

    target_link_libraries(
        path_validation_test
        PRIVATE

        control_core
        soem
        pthread
        m
    )

endif()

# ============================================================================
# GLOBAL STATE MACHINE UNIT TEST
# ============================================================================

add_executable(
    runtime_states_test
    StateMachine/Tests/test_runtime_states.c
    StateMachine/States/state_path_execution.c
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
    StateMachine/States/state_paused.c
    StateMachine/States/state_fault.c
    StateMachine/States/state_emergency_stop.c
)

target_compile_features(
    supervisor_task_compile_check
    PRIVATE
    c_std_11
)

target_include_directories(
    supervisor_task_compile_check
    PRIVATE

    StateMachine
    StateMachine/States

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    supervisor_task_compile_check
    PRIVATE

    freertos_kernel
    freertos_config
    control_core
)
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_interface_test.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/supervisor_mock_sim.cmake")