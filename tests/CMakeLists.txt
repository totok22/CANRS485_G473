cmake_minimum_required(VERSION 3.22)

project(CANRS485_G473_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(app_core_tests
    test_app_core.c
    ../Core/Src/app_command_parser.c
    ../Core/Src/app_fault.c
    ../Core/Src/app_utils.c
)

target_include_directories(app_core_tests PRIVATE
    stubs
    ../Core/Inc
)

target_compile_options(app_core_tests PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME app_core_tests COMMAND app_core_tests)
