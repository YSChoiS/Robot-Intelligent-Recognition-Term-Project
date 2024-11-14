cmake_minimum_required(VERSION 3.21)
project(CONSOLE-Gamepad)

add_library(${PROJECT_NAME}
        Gamepad.cpp
)
add_library(CONSOLE::Gamepad ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)