cmake_minimum_required(VERSION 3.20)

project(sensor_fusion
    VERSION 0.1.0
    DESCRIPTION "GNSS and IMU sensor fusion library"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sensor_fusion INTERFACE)
add_library(sensor_fusion::sensor_fusion ALIAS sensor_fusion)

target_include_directories(sensor_fusion INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(sensor_fusion INTERFACE cxx_std_20)

# Martix lib compilation
add_library(matrix SHARED src/matrix/matrix.cpp)
add_library(matrix::matrix ALIAS matrix)

target_include_directories(matrix PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(matrix PUBLIC cxx_std_20)

# Tests
enable_testing()

add_executable(matrix_tests tests/matrix_tests.cpp)
target_link_libraries(matrix_tests PRIVATE matrix)
add_test(NAME matrix_tests COMMAND matrix_tests)
