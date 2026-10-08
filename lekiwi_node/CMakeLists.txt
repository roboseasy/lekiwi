cmake_minimum_required(VERSION 3.8)
project(lekiwi_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(lekiwi_node_core
  src/base_controller.cpp
  src/feetech_motor_interface.cpp
  src/motor_readiness.cpp
  src/motor_safety_coordinator.cpp
  src/omni_kinematics.cpp
  src/odometry.cpp
)
target_include_directories(lekiwi_node_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(lekiwi_node_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

add_executable(lekiwi_node src/lekiwi_node.cpp)
target_link_libraries(lekiwi_node lekiwi_node_core)
ament_target_dependencies(lekiwi_node rclcpp)

install(TARGETS
  lekiwi_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  lekiwi_node_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_base_node_smoke
    test/test_base_node_smoke.cpp
  )
  target_link_libraries(test_base_node_smoke lekiwi_node_core)

  ament_add_gtest(test_omni_kinematics
    test/test_omni_kinematics.cpp
  )
  target_link_libraries(test_omni_kinematics lekiwi_node_core)

  ament_add_gtest(test_base_motor_interface_contract
    test/test_base_motor_interface_contract.cpp
  )
  target_link_libraries(test_base_motor_interface_contract lekiwi_node_core)

  ament_add_gtest(test_feetech_motor_interface
    test/test_feetech_motor_interface.cpp
  )
  target_link_libraries(test_feetech_motor_interface lekiwi_node_core)

  ament_add_gtest(test_motor_readiness
    test/test_motor_readiness.cpp
  )
  target_link_libraries(test_motor_readiness lekiwi_node_core)

  ament_add_gtest(test_motor_safety_coordinator
    test/test_motor_safety_coordinator.cpp
  )
  target_link_libraries(test_motor_safety_coordinator lekiwi_node_core)

  target_compile_definitions(test_base_node_smoke PRIVATE
    LEKIWI_NODE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

ament_package()
