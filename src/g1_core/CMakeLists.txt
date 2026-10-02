cmake_minimum_required(VERSION 3.8)

project(g1_core)

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )

endif()


# ============================================================
# Dependencias
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(unitree_hg REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_srvs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(unitree_api REQUIRED)


# ============================================================
# Mensajes propios
# ============================================================

rosidl_generate_interfaces(
  ${PROJECT_NAME}

  "msg/JointCommand.msg"
)

ament_export_dependencies(
  rosidl_default_runtime
)


# ============================================================
# Core antiguo
#
# No puede llamarse internamente "g1_core"
# porque ese nombre ya lo usa rosidl.
# ============================================================

add_executable(
  g1_core_node

  src/g1_core.cpp
  src/common/motor_crc_hg.cpp
)

# El archivo ejecutable final seguirá llamándose g1_core
set_target_properties(
  g1_core_node
  PROPERTIES
  OUTPUT_NAME g1_core
)


target_include_directories(
  g1_core_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/common
)


ament_target_dependencies(
  g1_core_node

  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  unitree_hg
)


# ============================================================
# Nuevo core low-level
# ============================================================

add_executable(
  g1_lowlevel_core

  src/g1_lowlevel_core.cpp
  src/common/motor_crc_hg.cpp
)


target_include_directories(
  g1_lowlevel_core
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/common
  /opt/unitree_ros2/example/src/include
)


ament_target_dependencies(
  g1_lowlevel_core

  rclcpp
  unitree_hg
  unitree_api
)

add_executable(
  g1_camera_bridge
  src/g1_camera_bridge.cpp
)

ament_target_dependencies(
  g1_camera_bridge
  rclcpp
  sensor_msgs
)

# ============================================================
# Typesupport para JointCommand
# ============================================================

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)


target_link_libraries(
  g1_lowlevel_core
  "${cpp_typesupport_target}"
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS
    g1_core_node
    g1_lowlevel_core
    g1_camera_bridge
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch

  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()