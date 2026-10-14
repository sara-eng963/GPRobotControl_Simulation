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

# Portable, nonblocking M7/M4 mailbox; hardware MPU mapping is separate.
add_library(intercore_ipc STATIC Intercore/ipc_mailbox.c)
target_compile_features(intercore_ipc PUBLIC c_std_11)
target_include_directories(intercore_ipc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Intercore)
add_executable(ipc_mailbox_test Intercore/Tests/test_ipc_mailbox.c)
target_link_libraries(ipc_mailbox_test PRIVATE intercore_ipc pthread)
target_compile_options(ipc_mailbox_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME ipc_mailbox_test COMMAND ipc_mailbox_test)
set_tests_properties(ipc_mailbox_test PROPERTIES TIMEOUT 15)


# Reusable STM32H745 target-only compile smoke test; independent of the POSIX
# kernel build. No HAL/board firmware or target hardware is required here.
find_program(GP_ARM_NONE_EABI_GCC arm-none-eabi-gcc)
find_program(GP_BASH_EXECUTABLE bash)

# Real FreeRTOS message-buffer transport, isolated from the host SIL Kit
# kernel configuration. The script builds a separate GCC_POSIX kernel.
if(GP_BASH_EXECUTABLE)
    add_test(NAME intercore_ipc_freertos_runtime_test
        COMMAND "${GP_BASH_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/Intercore/Tests/run_ipc_freertos_runtime.sh")
    set_tests_properties(intercore_ipc_freertos_runtime_test PROPERTIES
        TIMEOUT 90)
endif()
if(GP_ARM_NONE_EABI_GCC AND GP_BASH_EXECUTABLE)
    add_test(NAME intercore_ipc_arm_compile_test
        COMMAND "${GP_BASH_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/Intercore/Tests/test_ipc_freertos_compile.sh")
    set_tests_properties(intercore_ipc_arm_compile_test PROPERTIES
        TIMEOUT 60
        ENVIRONMENT "ARM_NONE_EABI_GCC=${GP_ARM_NONE_EABI_GCC}")
else()
    message(STATUS "IPC ARM compile test not registered (bash or arm-none-eabi-gcc unavailable)")
endif()

# HSEM adapter HAL/RTOS contract test. Uses isolated mocks, not physical IRQs.
if(GP_BASH_EXECUTABLE)
    add_test(NAME intercore_ipc_hsem_mock_test
        COMMAND "${GP_BASH_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/Intercore/Tests/run_ipc_hsem_mock.sh")
    set_tests_properties(intercore_ipc_hsem_mock_test PROPERTIES TIMEOUT 20)
endif()

# Real STM32CubeH7 header/driver cross-compilation. Opt-in because
# ST's separate SDK package is not vendored in this repository.
set(GP_STM32CUBEH7_ROOT "" CACHE PATH "Installed official STM32CubeH7 root")
if(GP_ARM_NONE_EABI_GCC AND GP_BASH_EXECUTABLE AND GP_STM32CUBEH7_ROOT
   AND EXISTS "${GP_STM32CUBEH7_ROOT}/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_hsem.h")
    add_test(NAME intercore_ipc_real_hal_compile_test
        COMMAND "${GP_BASH_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/Intercore/Tests/test_ipc_stm32_hal_compile.sh")
    set_tests_properties(intercore_ipc_real_hal_compile_test PROPERTIES
        TIMEOUT 90
        ENVIRONMENT
            "STM32CUBEH7_ROOT=${GP_STM32CUBEH7_ROOT};ARM_NONE_EABI_GCC=${GP_ARM_NONE_EABI_GCC}")
endif()

# M4 owns the flash producer/staging; M7 owns the 500 Hz consumer.
# Only GpTrajectoryShared crosses the inter-core SRAM boundary.
add_library(intercore_trajectory_stream STATIC Intercore/trajectory_stream.c)
target_compile_features(intercore_trajectory_stream PUBLIC c_std_11)
target_include_directories(intercore_trajectory_stream PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Intercore)
target_compile_options(intercore_trajectory_stream PRIVATE
    -Wall -Wextra -Werror=implicit-function-declaration
    -Werror=incompatible-pointer-types)
add_executable(intercore_trajectory_stream_test
    Intercore/Tests/test_trajectory_stream.c)
target_link_libraries(intercore_trajectory_stream_test PRIVATE
    intercore_trajectory_stream pthread)
target_compile_options(intercore_trajectory_stream_test PRIVATE
    -Wall -Wextra -Werror)
add_test(NAME intercore_trajectory_stream_test
    COMMAND intercore_trajectory_stream_test)
set_tests_properties(intercore_trajectory_stream_test
    PROPERTIES TIMEOUT 20)

# Verify the actual PathExecutionServices callback ABI can consume the
# M7-owned side of the cross-core stream. Existing SIL Kit path unchanged.
add_library(intercore_trajectory_path_port STATIC
    Intercore/trajectory_stream_path_port.c)
target_link_libraries(intercore_trajectory_path_port PUBLIC
    intercore_trajectory_stream)
target_compile_features(intercore_trajectory_path_port PUBLIC c_std_11)
target_compile_options(intercore_trajectory_path_port PRIVATE
    -Wall -Wextra -Werror)

add_executable(intercore_trajectory_path_port_test
    Intercore/Tests/test_trajectory_stream_path_port.c)
target_link_libraries(intercore_trajectory_path_port_test PRIVATE
    intercore_trajectory_path_port)
target_compile_options(intercore_trajectory_path_port_test PRIVATE
    -Wall -Wextra -Werror)
add_test(NAME intercore_trajectory_path_port_test
    COMMAND intercore_trajectory_path_port_test)
set_tests_properties(intercore_trajectory_path_port_test
    PROPERTIES TIMEOUT 15)


# B5: standalone transport-neutral status/command policy on the host.
# The firmware uses the same C unit from CM4/Application/Intercore only.
add_executable(intercore_hmi_lifecycle_test
    Intercore/hmi_lifecycle.c
    Intercore/Tests/test_hmi_lifecycle.c)
target_compile_features(intercore_hmi_lifecycle_test PRIVATE c_std_11)
target_compile_options(intercore_hmi_lifecycle_test PRIVATE
    -Wall -Wextra -Werror)
add_test(NAME intercore_hmi_lifecycle_test
    COMMAND intercore_hmi_lifecycle_test)
set_tests_properties(intercore_hmi_lifecycle_test
    PROPERTIES TIMEOUT 20)

# B4: canonical storage worker/request library, host-only fake tests.
add_library(intercore_trajectory_storage STATIC
    Intercore/trajectory_storage_service.c)
target_compile_features(intercore_trajectory_storage PUBLIC c_std_11)
target_link_libraries(intercore_trajectory_storage PUBLIC
    intercore_trajectory_stream)
target_compile_options(intercore_trajectory_storage PRIVATE
    -Wall -Wextra -Werror)
add_executable(intercore_trajectory_storage_test
    Intercore/Tests/test_trajectory_storage_service.c)
target_link_libraries(intercore_trajectory_storage_test PRIVATE
    intercore_trajectory_storage intercore_validation_storage)
target_compile_options(intercore_trajectory_storage_test PRIVATE
    -Wall -Wextra -Werror)
add_test(NAME intercore_trajectory_storage_test
    COMMAND intercore_trajectory_storage_test)
set_tests_properties(intercore_trajectory_storage_test
    PROPERTIES TIMEOUT 20)
target_link_libraries(intercore_trajectory_path_port PUBLIC
    intercore_trajectory_storage intercore_validation_storage)

# Separate M7 validation math from the M4-owned QSPI/NOR storage.
add_library(intercore_validation_storage STATIC
    Intercore/validation_storage_bridge.c)
target_compile_features(intercore_validation_storage PUBLIC c_std_11)
target_include_directories(intercore_validation_storage PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Intercore)
target_compile_options(intercore_validation_storage PRIVATE
    -Wall -Wextra -Werror)

add_executable(intercore_validation_storage_test
    Intercore/Tests/test_validation_storage_bridge.c)
target_link_libraries(intercore_validation_storage_test PRIVATE
    intercore_validation_storage pthread)
target_compile_options(intercore_validation_storage_test PRIVATE
    -Wall -Wextra -Werror)
add_test(NAME intercore_validation_storage_test
    COMMAND intercore_validation_storage_test)
set_tests_properties(intercore_validation_storage_test
    PROPERTIES TIMEOUT 15)

# Host-only timing CSV recorder. Written by a background pthread, never
# from Supervisor/CAN tasks. This library is NOT STM32 firmware code.
add_library(host_timing_trace STATIC Simulation/Timing/host_timing_trace.c)
target_compile_features(host_timing_trace PUBLIC c_std_11)
target_compile_options(host_timing_trace PRIVATE -Wall -Wextra -Werror)
target_link_libraries(host_timing_trace PUBLIC pthread)
add_executable(host_timing_trace_test
    Simulation/Timing/test_host_timing_trace.c)
target_include_directories(host_timing_trace_test PRIVATE Simulation/Timing)
target_link_libraries(host_timing_trace_test PRIVATE host_timing_trace)
target_compile_options(host_timing_trace_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME host_timing_trace_test COMMAND host_timing_trace_test)
set_tests_properties(host_timing_trace_test PROPERTIES TIMEOUT 10)

# Python analyzer smoke test uses a tiny synthetic fixture and checks the
# summary reports the expected misses, not hardware timing measurements.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME host_timing_analyzer_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/Simulation/Timing/test_analyzer.py
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/analyze_timing_csv.py)
    set_tests_properties(host_timing_analyzer_test PROPERTIES TIMEOUT 10)
endif()

# FreeRTOS runs with its existing POSIX host port.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)
# Host simulation needs space for the dedicated non-real-time flash writer.
# This definition is NOT enabled in the STM32 firmware/toolchain.
if(ENABLE_SILKIT)
    target_compile_definitions(freertos_config INTERFACE GP_POSIX_ASYNC_VALIDATION_HEAP=1)
endif()
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
target_link_libraries(path_validation_test PRIVATE control_core avatar_drives intercore_validation_storage m)
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
    intercore_ipc
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
