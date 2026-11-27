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

# Tests
enable_testing()

add_executable(matrix_tests tests/matrix_tests.cpp)
target_link_libraries(matrix_tests PRIVATE sensor_fusion)
add_test(NAME matrix_tests COMMAND matrix_tests)

add_executable(nmea_tests tests/nmea_tests.cpp)
target_link_libraries(nmea_tests PRIVATE sensor_fusion)
add_test(NAME nmea_tests COMMAND nmea_tests)

add_executable(imu_tests tests/imu_tests.cpp)
target_link_libraries(imu_tests PRIVATE sensor_fusion)
add_test(NAME imu_tests COMMAND imu_tests)

add_executable(scenario_tests tests/scenario_tests.cpp)
target_link_libraries(scenario_tests PRIVATE sensor_fusion)
add_test(NAME scenario_tests COMMAND scenario_tests)

add_executable(synthetic_drive_tests tests/synthetic_drive_tests.cpp)
target_link_libraries(synthetic_drive_tests PRIVATE sensor_fusion)
add_test(NAME synthetic_drive_tests COMMAND synthetic_drive_tests)

add_executable(ekf_tests tests/ekf_tests.cpp)
target_link_libraries(ekf_tests PRIVATE sensor_fusion)
add_test(NAME ekf_tests COMMAND ekf_tests)

add_executable(gnss_csv_tests tests/gnss_csv_tests.cpp)
target_link_libraries(gnss_csv_tests PRIVATE sensor_fusion)
add_test(NAME gnss_csv_tests COMMAND gnss_csv_tests)

add_executable(kitti_scenario_tests tests/kitti_scenario_tests.cpp)
target_link_libraries(kitti_scenario_tests PRIVATE sensor_fusion)
add_test(NAME kitti_scenario_tests COMMAND kitti_scenario_tests)

add_executable(metrics_tests tests/metrics_tests.cpp)
target_link_libraries(metrics_tests PRIVATE sensor_fusion)
add_test(NAME metrics_tests COMMAND metrics_tests)

# Reports
add_executable(kitti_run apps/report/kitti_run.cpp)
target_link_libraries(kitti_run PRIVATE sensor_fusion)
add_test(NAME kitti_run_tests
    COMMAND ${CMAKE_COMMAND}
        -DKITTI_RUN=$<TARGET_FILE:kitti_run>
        -DWORK=${CMAKE_CURRENT_BINARY_DIR}/kitti_run_tests
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/kitti_run_tests.cmake)
