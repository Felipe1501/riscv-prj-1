cmake_minimum_required(VERSION 3.16)

project(riscv_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(riscv_sim
    src/main.cpp
    src/memory.cpp
    src/bus.cpp
    src/cpu.cpp
    src/utils.cpp
)