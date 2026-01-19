cmake_minimum_required(VERSION 3.8)
project(unitree_interface)

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

############ IK ############
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${pinocchio_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR}
    # ${CMAKE_CURRENT_SOURCE_DIR}
)

# Source files
set(IK_SOURCES
    src/arm/weighted_moving_filter.cpp
    src/arm/robot_arm_ik.cpp
    src/arm/robot_arm_ik_g1_23dof.cpp
)

# Headers
set(IK_HEADERS
    src/arm/include/weighted_moving_filter.h
    src/arm/include/robot_arm_ik.h
    src/arm/include/robot_arm_ik_g1_23dof.h
)
########################

# unitree_sdk2
add_subdirectory(third_party/unitree_sdk2)

# Generate service definitions
# set(msg_files
#   ""
# )

set(srv_files
  "interfaces/srv/ChangeControlMode.srv"
)

rosidl_generate_interfaces(unitree_interface
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target
  unitree_interface rosidl_typesupport_cpp)

# Create the library
add_library(unitree_interface_lib
  STATIC
    src/unitree_interface.cpp
    src/control_modes.cpp
    src/unitree_sdk_wrapper.cpp
    src/mode_transitions.cpp
    ${IK_SOURCES}
    ${IK_HEADERS}
)

target_include_directories(unitree_interface_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Not decreasing compile time for some reason
# target_precompile_headers(unitree_interface_lib
#   PRIVATE
#     include/unitree_interface/pch.hpp
# )

target_link_libraries(unitree_interface_lib
  "${typesupport_target}"
  unitree_sdk2
    ${EIGEN3_LIBRARIES}
    pinocchio::pinocchio
    casadi

)

ament_target_dependencies(unitree_interface_lib
  rclcpp
  geometry_msgs
)

# Create the executable
add_executable(unitree_interface_node
  src/unitree_interface_node.cpp
)

target_link_libraries(unitree_interface_node
  unitree_interface_lib
)

install(TARGETS unitree_interface_node
  DESTINATION lib/unitree_interface
)

install(DIRECTORY launch
  DESTINATION share/unitree_interface
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test control_modes
  ament_add_gtest(test_control_modes
    test/test_control_modes.cpp
  )

  target_link_libraries(test_control_modes
    unitree_interface_lib
  )

  # Test unitree_interface
  ament_add_gtest(test_unitree_interface
    test/test_unitree_interface.cpp
  )

  target_link_libraries(test_unitree_interface
    unitree_interface_lib
  )
endif()

ament_package()
