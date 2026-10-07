cmake_minimum_required(VERSION 3.16)

project(AvatarM_CAN_GUI C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

add_subdirectory(
    "${REPO_ROOT}/external/raylib"
    "${CMAKE_BINARY_DIR}/raylib"
)

add_executable(
    avatar_m_can_testbench

    avatar_m_can_testbench.c
    sim_can_bus.c

    "${REPO_ROOT}/CANComm/CANopen/canopen_master.c"
    "${REPO_ROOT}/CANComm/CANopen/canopen_nmt.c"
    "${REPO_ROOT}/CANComm/CANopen/canopen_sdo.c"
    "${REPO_ROOT}/CANComm/CANopen/canopen_heartbeat.c"

    "${REPO_ROOT}/ServoDrive/CiA402/cia402.c"

    "${REPO_ROOT}/ServoDrive/AvatarM/avatar_m_pdo.c"
    "${REPO_ROOT}/ServoDrive/AvatarM/avatar_m_drive.c"
    "${REPO_ROOT}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c"
)

target_include_directories(
    avatar_m_can_testbench
    PRIVATE

    "${REPO_ROOT}"
    "${REPO_ROOT}/CANComm"
    "${REPO_ROOT}/CANComm/CANopen"
    "${REPO_ROOT}/ServoDrive/CiA402"
    "${REPO_ROOT}/ServoDrive/AvatarM"
    "${REPO_ROOT}/ServoDrive/AvatarM/Sim"
)

target_link_libraries(
    avatar_m_can_testbench
    PRIVATE

    raylib
    m
)
