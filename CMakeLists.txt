cmake_minimum_required(VERSION 3.8)
project(fieldfriend_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fieldfriend_driver_node
  src/fieldfriend_driver_node.cpp
  src/serial_communication.cpp
  src/data_odom.cpp
  src/data_bms.cpp
  src/odom_handler.cpp
  src/bms_handler.cpp
  src/twist_handler.cpp
  src/estop_handler.cpp
  src/estop_button_handler.cpp
  src/yaxis_handler.cpp
  src/zaxis_handler.cpp
  src/configuration_handler.cpp
)

target_include_directories(fieldfriend_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fieldfriend_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS fieldfriend_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
