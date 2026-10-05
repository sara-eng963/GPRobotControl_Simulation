cmake_minimum_required(VERSION 3.16)

project(AvatarM_Drive C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(
    avatar_m_drive_test

    Tests/test_avatar_m_drive.c
    avatar_m_drive.c
    avatar_m_pdo.c

    ${REPO_ROOT}/ServoDrive/CiA402/cia402.c

    ${REPO_ROOT}/CANComm/CANopen/canopen_nmt.c
    ${REPO_ROOT}/CANComm/CANopen/canopen_sdo.c
    ${REPO_ROOT}/CANComm/CANopen/canopen_heartbeat.c
)

target_include_directories(
    avatar_m_drive_test
    PRIVATE

    ${REPO_ROOT}
    ${REPO_ROOT}/CANComm
    ${REPO_ROOT}/CANComm/CANopen
    ${REPO_ROOT}/ServoDrive/CiA402
    ${CMAKE_CURRENT_LIST_DIR}
)
