cmake_minimum_required(VERSION 3.8)
project(lekiwi_navigation2)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME test_navigation_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_navigation_contract.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

endif()

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
