cmake_minimum_required(VERSION 3.16)
project(my_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Default to release build
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crisp_controllers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(parameter_traits REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(TARGET Eigen3::Eigen)
  set(MY_CONTROLLER_EIGEN_TARGET Eigen3::Eigen)
else()
  set(MY_CONTROLLER_EIGEN_TARGET Eigen3::Eigen3)
endif()

generate_parameter_library(
  my_controller_parameters
  src/my_controller.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/my_controller.cpp
  src/impedance_controller.cpp
  src/home_position_client.cpp
)

add_executable(impedance_controller_test
  src/test.cpp
)
set_target_properties(impedance_controller_test PROPERTIES OUTPUT_NAME test)

add_executable(franka_home_position
  src/home_position.cpp
)
set_target_properties(franka_home_position PROPERTIES OUTPUT_NAME home_position)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${MY_CONTROLLER_EIGEN_TARGET}
  PRIVATE
    my_controller_parameters
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    generate_parameter_library
    std_msgs
    crisp_controllers
    geometry_msgs
    rcl_interfaces
    controller_manager_msgs
    tf2
    parameter_traits
    rclcpp_action
    control_msgs
    trajectory_msgs
)

target_link_libraries(impedance_controller_test
  PRIVATE
    ${PROJECT_NAME}
)

target_link_libraries(franka_home_position
  PRIVATE
    ${PROJECT_NAME}
)

ament_target_dependencies(impedance_controller_test
  PUBLIC
    rclcpp
    geometry_msgs
    controller_manager_msgs
    tf2
)

ament_target_dependencies(franka_home_position
  PUBLIC
    rclcpp
)

pluginlib_export_plugin_description_file(controller_interface my_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS impedance_controller_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS franka_home_position
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  generate_parameter_library
  std_msgs
  crisp_controllers
  geometry_msgs
  rcl_interfaces
  controller_manager_msgs
  tf2
  Eigen3
  parameter_traits
  rclcpp_action
  control_msgs
  trajectory_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
