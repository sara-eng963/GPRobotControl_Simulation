cmake_minimum_required(VERSION 3.16)

project(ControlCore C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

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
    Trajectory/circular_path.c
    Trajectory/arc_length_parameterize.c
    Trajectory/s_curve_time_scaling.c
    Trajectory/s_curve_profile.c

    Execution/trajectory_buffer.c

    Pipeline/single_segment_line.c
    Pipeline/single_segment_line_stream.c
    Pipeline/single_segment_circular.c
    Pipeline/single_segment_circular_stream.c
)

target_include_directories(
    control_core
    PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/Math
    ${CMAKE_CURRENT_SOURCE_DIR}/Kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory
    ${CMAKE_CURRENT_SOURCE_DIR}/Execution
    ${CMAKE_CURRENT_SOURCE_DIR}/Pipeline
)

target_link_libraries(
    control_core
    PUBLIC
    m
)

target_compile_options(
    control_core
    PRIVATE

    -Wall
    -Wextra
    -Wpedantic
)

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

add_executable(
    single_segment_circular_test
    Tests/single_segment_circular_test.c
)

target_link_libraries(
    single_segment_circular_test
    PRIVATE
    control_core
    m
)

add_executable(
    trajectory_buffer_test
    Tests/trajectory_buffer_test.c
)

target_link_libraries(
    trajectory_buffer_test
    PRIVATE
    control_core
    m
)

add_executable(
    s_curve_profile_test
    Tests/s_curve_profile_test.c
)

target_link_libraries(
    s_curve_profile_test
    PRIVATE
    control_core
    m
)

add_executable(
    single_segment_line_stream_test
    Tests/single_segment_line_stream_test.c
)

target_link_libraries(
    single_segment_line_stream_test
    PRIVATE
    control_core
    m
)

# ============================================================================
# BUFFERED STREAMING TRAJECTORY TEST
# ============================================================================

add_executable(
    buffered_stream_test
    Tests/buffered_stream_test.c
)

target_link_libraries(
    buffered_stream_test
    PRIVATE
    control_core
    m
)

# ============================================================================
# STREAMING PLANNER PERFORMANCE BENCHMARK
# ============================================================================

add_executable(
    streaming_planner_benchmark
    Tests/streaming_planner_benchmark.c
)

target_link_libraries(
    streaming_planner_benchmark
    PRIVATE
    control_core
    m
)
