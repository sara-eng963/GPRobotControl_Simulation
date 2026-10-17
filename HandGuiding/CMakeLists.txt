# Build independently for focused Stage 4 tests, or from the repository root.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)
    project(HandGuiding C)
    enable_testing()
endif()

add_library(hand_guiding STATIC
    ft_sensor.c
    sensor_config.c
    admittance_control.c
    cartesian_joint_mapper.c
    wrench_frame.c
    hand_guiding_pipeline.c
)
target_compile_features(hand_guiding PUBLIC c_std_11)
target_include_directories(hand_guiding PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(hand_guiding PRIVATE -Wall -Wextra -Wpedantic)
# Standalone tests build ONLY the ControlCore units used by this mapper.
# Root builds instead reuse the real repository control_core target.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_library(hand_guiding_control_core STATIC
        ../ControlCore/Kinematics/dh_transform.c
        ../ControlCore/Kinematics/control_jacobian.c
        ../ControlCore/Kinematics/control_fk.c
        ../ControlCore/Math/matrix6.c
        ../ControlCore/Analysis/singularity.c
        ../ControlCore/Config/robot_config.c
    )
    target_compile_features(hand_guiding_control_core PUBLIC c_std_11)
    target_compile_options(hand_guiding_control_core PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(hand_guiding PUBLIC hand_guiding_control_core m)
else()
    target_link_libraries(hand_guiding PUBLIC control_core m)
endif()

foreach(test_name IN ITEMS test_ft_sensor test_sensor_config test_wrench_processing test_admittance_control test_cartesian_joint_mapper test_wrench_frame test_hand_guiding_pipeline)
    add_executable(${test_name} Tests/${test_name}.c)
    target_link_libraries(${test_name} PRIVATE hand_guiding)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 10)
endforeach()
