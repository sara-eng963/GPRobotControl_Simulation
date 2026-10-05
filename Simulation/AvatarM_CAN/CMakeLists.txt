cmake_minimum_required(VERSION 3.28)

project(AvatarM_SILKit_Integration C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

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
        "SILKIT_ROOT must point to the SIL Kit installation root"
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

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

set(
    AVATAR_M_SILKIT_COMMON_SOURCES

    avatar_m_silkit_node.cpp

    ${REPO_ROOT}/CANComm/SILKit/silkit_can_backend.cpp

    ${REPO_ROOT}/ServoDrive/AvatarM/Sim/avatar_m_node_sim.c
    ${REPO_ROOT}/ServoDrive/AvatarM/avatar_m_pdo.c
)

function(configure_avatar_m_silkit_test target_name)
    target_include_directories(
        ${target_name}
        PRIVATE

        ${REPO_ROOT}
        ${REPO_ROOT}/CANComm
        ${REPO_ROOT}/CANComm/CANopen
        ${REPO_ROOT}/CANComm/SILKit
        ${REPO_ROOT}/ServoDrive/AvatarM
        ${REPO_ROOT}/ServoDrive/AvatarM/Sim
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(
        ${target_name}
        PRIVATE

        SilKit::SilKit
        Threads::Threads
    )
endfunction()

add_executable(
    avatar_m_silkit_integration_test

    test_avatar_m_silkit_node.cpp
    ${AVATAR_M_SILKIT_COMMON_SOURCES}
)

configure_avatar_m_silkit_test(
    avatar_m_silkit_integration_test
)

add_executable(
    avatar_m_silkit_six_node_test

    test_avatar_m_silkit_six_nodes.cpp
    ${AVATAR_M_SILKIT_COMMON_SOURCES}
)

configure_avatar_m_silkit_test(
    avatar_m_silkit_six_node_test
)
