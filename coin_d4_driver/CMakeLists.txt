################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(coin_d4_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(LIBRARY_SRC
  src/lidar_sdk/lidar_data_processor.cpp
  src/lidar_sdk/mtime.cpp
  src/lidar_sdk/serial_port.cpp
  src/lidar_sdk/timer.cpp
  src/coin_d4_base_handler.cpp
  src/coin_d4_lifecycle_handler.cpp
  src/coin_d4_node_handler.cpp)

add_library(${PROJECT_NAME}_lib SHARED ${LIBRARY_SRC})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(single_coin_d4_node src/single_coin_d4_node.cpp)
add_executable(multi_coin_d4_node src/multi_coin_d4_node.cpp)

target_link_libraries(${PROJECT_NAME}_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

target_link_libraries(single_coin_d4_node
  ${PROJECT_NAME}_lib
)
target_link_libraries(multi_coin_d4_node
  ${PROJECT_NAME}_lib
)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS single_coin_d4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS multi_coin_d4_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp_lifecycle)

ament_package()
