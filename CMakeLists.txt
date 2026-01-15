cmake_minimum_required(VERSION 3.8)
project(joint_cmd_mux)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# unitree_sdk2
add_subdirectory(third_party/unitree_sdk2)

# Generate service definitions
# set(msg_files
#   ""
# )

set(srv_files
  "interfaces/srv/ChangeControlMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Create the library
add_library(joint_cmd_mux_lib
  STATIC
    src/joint_cmd_mux.cpp
    src/control_modes.cpp
    src/unitree_interface.cpp
)

target_include_directories(joint_cmd_mux_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_precompile_headers(joint_cmd_mux_lib
  PRIVATE
    include/${CMAKE_PROJECT_NAME}/pch.hpp
)

target_link_libraries(joint_cmd_mux_lib
  "${typesupport_target}"
  unitree_sdk2
)

ament_target_dependencies(joint_cmd_mux_lib
  rclcpp
  geometry_msgs
)

# Create the executable
add_executable(joint_cmd_mux_node
  src/joint_cmd_mux_node.cpp
)

target_link_libraries(joint_cmd_mux_node
  joint_cmd_mux_lib
)

install(TARGETS joint_cmd_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test control_modes
  ament_add_gtest(test_control_modes
    test/test_control_modes.cpp
  )

  target_link_libraries(test_control_modes
    joint_cmd_mux_lib
  )

  # Test joint_cmd_mux
  ament_add_gtest(test_joint_cmd_mux
    test/test_joint_cmd_mux.cpp
  )

  target_link_libraries(test_joint_cmd_mux
    joint_cmd_mux_lib
  )
endif()

ament_package()
