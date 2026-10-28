cmake_minimum_required(VERSION 3.20)
project(ipc_freertos_posix_test C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build a SEPARATE kernel instance: the main SIL Kit kernel has static
# allocation disabled. Never link a target with a mismatched config.
if(NOT DEFINED FREERTOS_KERNEL_ROOT)
    get_filename_component(FREERTOS_KERNEL_ROOT
        "${CMAKE_CURRENT_LIST_DIR}/../../../../FreeRTOS-Kernel" ABSOLUTE)
endif()
if(NOT EXISTS "${FREERTOS_KERNEL_ROOT}/include/FreeRTOS.h")
    message(FATAL_ERROR
        "FreeRTOS-Kernel not found at '${FREERTOS_KERNEL_ROOT}'. "
        "Set FREERTOS_KERNEL_ROOT to your local kernel checkout.")
endif()

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/config")
target_compile_definitions(freertos_config INTERFACE projCOVERAGE_TEST=0)

set(FREERTOS_PORT GCC_POSIX CACHE STRING "" FORCE)
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)
add_subdirectory("${FREERTOS_KERNEL_ROOT}"
    "${CMAKE_CURRENT_BINARY_DIR}/freertos-kernel")

find_package(Threads REQUIRED)
add_executable(ipc_freertos_runtime_test
    "${CMAKE_CURRENT_LIST_DIR}/../test_ipc_freertos_runtime.c"
    "${CMAKE_CURRENT_LIST_DIR}/../../ipc_mailbox_freertos.c")
target_compile_definitions(ipc_freertos_runtime_test PRIVATE
    IPC_FREERTOS_BACKEND=1
    IPC_POSIX_TEST=1)
target_include_directories(ipc_freertos_runtime_test PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../..")
target_link_libraries(ipc_freertos_runtime_test PRIVATE
    freertos_kernel freertos_config Threads::Threads)
target_compile_options(ipc_freertos_runtime_test PRIVATE
    -Wall -Wextra -Werror)
