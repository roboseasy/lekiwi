cmake_minimum_required(VERSION 3.8)
project(lekiwi_sensors LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(bosch_bmi160_vendor STATIC
  vendor/bosch_bmi160/bmi160.c
)
target_include_directories(bosch_bmi160_vendor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/bosch_bmi160>
)
set_target_properties(bosch_bmi160_vendor PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON
)

add_executable(bmi160_node
  src/bmi160_node.cpp
)
target_compile_features(bmi160_node PUBLIC cxx_std_17)
target_include_directories(bmi160_node PRIVATE
  include
  vendor/bosch_bmi160
)
target_link_libraries(bmi160_node bosch_bmi160_vendor)
ament_target_dependencies(bmi160_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  add_test(NAME test_navigation_scan_filter
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_scan_filter.py)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_bmi160_conversion
    test/test_bmi160_conversion.cpp
  )
  target_compile_features(test_bmi160_conversion PUBLIC cxx_std_17)
  target_include_directories(test_bmi160_conversion PRIVATE include)

  add_test(
    NAME test_bmi160_distribution
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_bmi160_distribution.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_test(
    NAME test_scan_tf_timing_probe
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scan_tf_timing_probe.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_test(
    NAME test_ydlidar_params
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ydlidar_params.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_test(
    NAME test_scan_timing_normalizer
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scan_timing_normalizer.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  TARGETS bmi160_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES
    vendor/bosch_bmi160/LICENSE
    vendor/bosch_bmi160/VENDOR_SOURCE.md
  DESTINATION share/${PROJECT_NAME}/vendor/bosch_bmi160
)

install(
  PROGRAMS
    scripts/navigation_scan_filter
    scripts/scan_tf_timing_probe
    scripts/scan_timing_normalizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
