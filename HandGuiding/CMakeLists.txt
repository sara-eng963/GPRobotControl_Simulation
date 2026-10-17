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
)
target_compile_features(hand_guiding PUBLIC c_std_11)
target_include_directories(hand_guiding PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(hand_guiding PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hand_guiding PUBLIC m)

foreach(test_name IN ITEMS test_ft_sensor test_sensor_config test_wrench_processing)
    add_executable(${test_name} Tests/${test_name}.c)
    target_link_libraries(${test_name} PRIVATE hand_guiding)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 10)
endforeach()
