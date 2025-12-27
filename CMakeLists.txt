cmake_minimum_required(VERSION 3.20)
project(MAPF_TW_Project)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -fPIC)

# 1. Tìm Python (để biết đang dùng python nào)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 2. Add Vendor (Pybind11)
# Nếu ông clone code pybind11 vào vendor/pybind11 rồi thì dòng này sẽ cấu hình nó
add_subdirectory(vendor/pybind11)

# 3. Add Core C++ (Thằng này phải chạy trước bindings)
set(MAPFSOLVER_BUILD_EXE OFF CACHE BOOL "Do not Build MAPFSolver executable" FORCE)
add_subdirectory(core/MAPFSolver)

# 4. Add Bindings (Thằng này chạy sau cùng)
add_subdirectory(core/bindings)