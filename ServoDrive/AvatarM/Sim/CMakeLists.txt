cmake_minimum_required(VERSION 3.16)
project(avatar_m_node_sim_test C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(
    avatar_m_node_sim_test

    test_avatar_m_node_sim.c
    avatar_m_node_sim.c
    ../avatar_m_pdo.c
)

target_include_directories(
    avatar_m_node_sim_test
    PRIVATE

    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../../../CANComm
    ${CMAKE_CURRENT_LIST_DIR}/../../../CANComm/CANopen
)
