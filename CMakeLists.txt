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

# ----------------------------
# Our application
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
# MOCK HMI
# ============================================================================

add_executable(
    mock_hmi
    hmi.c
)

target_compile_features(
    mock_hmi
    PRIVATE
    c_std_11
)

# Force-include the desktop view controls only for the HMI target.
# This adds mouse-wheel/keyboard zoom and middle-button panning while leaving
# the HMI layout, UDP protocol and controller behavior unchanged.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
        mock_hmi
        PRIVATE
        -include
        ${CMAKE_CURRENT_SOURCE_DIR}/hmi_zoom_hooks.h
    )
endif()

target_link_libraries(
    mock_hmi
    PRIVATE
    raylib
)

# ============================================================================
# CONTROL CORE
# ============================================================================

add_subdirectory(ControlCore)

target_link_libraries(
    freertos_pc_test
    PRIVATE
    control_core
)
