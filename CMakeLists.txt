cmake_minimum_required(VERSION 3.28)

project(freertos_pc_test C)

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

# ----------------------------
# FreeRTOS / EtherCAT application
# ----------------------------

add_executable(
    freertos_pc_test

    main.c

    EtherCATComm/ethercat_master.c
    EtherCATComm/SOEM/soem_backend.c

    ServoDrive/CiA402/cia402.c

    ServoDrive/A6EC/a6ec_pdo.c
    ServoDrive/A6EC/a6ec_drive.c
)

target_include_directories(
    freertos_pc_test
    PRIVATE

    EtherCATComm
    EtherCATComm/SOEM

    ServoDrive/CiA402
    ServoDrive/A6EC
)

target_link_libraries(
    freertos_pc_test
    PRIVATE
    freertos_kernel
    freertos_config
    soem
    pthread
    control_core
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

    hmi.c

    HMI/hmi_app.c
    HMI/hmi_protocol.c
    HMI/hmi_theme.c
    HMI/hmi_runtime_guard.c
    HMI/waypoint_editor_3d.c
    HMI/workspace_monitor.c
)

target_compile_features(
    mock_hmi
    PRIVATE
    c_std_11
)

target_include_directories(
    mock_hmi
    PRIVATE
    HMI
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
