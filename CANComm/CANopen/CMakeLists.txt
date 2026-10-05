cmake_minimum_required(VERSION 3.28)

project(CANopen_Management_Tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(
    canopen_management_test

    ${REPO_ROOT}/CANComm/Tests/test_canopen_management.c

    ${CMAKE_CURRENT_LIST_DIR}/canopen_nmt.c
    ${CMAKE_CURRENT_LIST_DIR}/canopen_sdo.c
    ${CMAKE_CURRENT_LIST_DIR}/canopen_heartbeat.c
)

target_include_directories(
    canopen_management_test
    PRIVATE

    ${REPO_ROOT}/CANComm
    ${CMAKE_CURRENT_LIST_DIR}
)
