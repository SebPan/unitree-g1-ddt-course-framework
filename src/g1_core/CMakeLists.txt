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

find_package(std_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)

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
  /opt/unitree_ros2/examples/src/include
)



target_include_directories(g1_core_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  g1_core_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  unitree_hg
  unitree_api
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
  g1_core_node
  "${cpp_typesupport_target}"
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS
    g1_core_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# ============================================================
# High Level Core - Python
# ============================================================

install(
  PROGRAMS
    scripts/g1_high_core.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    g1_high_core
)

ament_package()
