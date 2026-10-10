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
