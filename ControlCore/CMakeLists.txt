cmake_minimum_required(VERSION 3.16)

project(ControlCore C)


# ============================================================================
# C STANDARD
# ============================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


# ============================================================================
# CONTROL CORE LIBRARY
# ============================================================================

add_library(
    control_core
    STATIC

    Config/robot_config.c

    Math/math3d.c
    Math/matrix6.c

    Kinematics/dh_transform.c
    Kinematics/control_fk.c
    Kinematics/control_jacobian.c
    Kinematics/adls_ik.c

    Trajectory/generate_line_waypoints.c
    Trajectory/arc_length_parameterize.c
    Trajectory/s_curve_time_scaling.c

    Pipeline/single_segment_line.c
)


# ============================================================================
# INCLUDE DIRECTORIES
# ============================================================================

target_include_directories(
    control_core
    PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/Math
    ${CMAKE_CURRENT_SOURCE_DIR}/Kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory
    ${CMAKE_CURRENT_SOURCE_DIR}/Pipeline
)


# ============================================================================
# MATH LIBRARY
# ============================================================================

target_link_libraries(
    control_core
    PUBLIC
    m
)


# ============================================================================
# WARNINGS
# ============================================================================

target_compile_options(
    control_core
    PRIVATE

    -Wall
    -Wextra
    -Wpedantic
)


# ============================================================================
# SINGLE-SEGMENT LINE TEST
# ============================================================================

add_executable(
    single_segment_line_test

    Tests/single_segment_line_test.c
)


target_link_libraries(
    single_segment_line_test
    PRIVATE

    control_core
    m
)
