cmake_minimum_required(VERSION 3.8)
project(lekiwi_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} SKIP_COMPILE)

option(LEKIWI_INSTALL_DIAGNOSTICS "Install developer topic diagnostics" OFF)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_hardware_launch_args test/test_hardware_launch_args.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV PYTHONDONTWRITEBYTECODE=1
  )
  add_test(
    NAME test_base_param_contracts
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_base_param_contracts.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_launch_contracts test/test_launch_contracts.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    ENV PYTHONDONTWRITEBYTECODE=1
  )
  add_test(
    NAME test_install_layout
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_install_layout.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(
    NAME test_topic_services
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test/diagnostics -q
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(LEKIWI_INSTALL_DIAGNOSTICS)
  install(PROGRAMS
    tools/diagnostics/test_ros_topics.py
    tools/diagnostics/test_service_server.py
    tools/diagnostics/test_all.py
    DESTINATION lib/${PROJECT_NAME})
  install(FILES tools/diagnostics/topic_test.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
endif()

ament_package()
