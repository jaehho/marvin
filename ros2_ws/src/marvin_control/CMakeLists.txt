cmake_minimum_required(VERSION 3.22)
project(marvin_control LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

# ##############################################################################
# C++ Libraries ##
# ##############################################################################

# This library provides a way of loading parameters for servo
generate_parameter_library(moveit_servo_lib_parameters
                           config/servo_parameters.yaml)

# This library provides a C++ interface for sending realtime twist or joint
# commands to a robot
add_library(
  moveit_servo_lib_cpp SHARED src/collision_monitor.cpp src/servo.cpp
                              src/utils/common.cpp src/utils/command.cpp)
set_target_properties(moveit_servo_lib_cpp PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_cpp moveit_servo_lib_parameters)
ament_target_dependencies(moveit_servo_lib_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(moveit_servo_lib_ros SHARED src/servo_node.cpp)
set_target_properties(moveit_servo_lib_ros PROPERTIES VERSION
                                                      "${moveit_servo_VERSION}")
target_link_libraries(moveit_servo_lib_ros moveit_servo_lib_cpp)
ament_target_dependencies(moveit_servo_lib_ros ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ##############################################################################
# Components ##
# ##############################################################################

rclcpp_components_register_node(moveit_servo_lib_ros PLUGIN
                                "moveit_servo::ServoNode" EXECUTABLE servo_node)

# ##############################################################################
# Executable Nodes ##
# ##############################################################################

# Executable node for the joint jog demo
add_executable(demo_joint_jog executables/demo_joint_jog.cpp)
target_link_libraries(demo_joint_jog moveit_servo_lib_cpp)
ament_target_dependencies(demo_joint_jog ${THIS_PACKAGE_INCLUDE_DEPENDS})

#HELLO MOVEIT
add_executable(hello_moveit executables/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

#ORIGINAL PUBLISHER
add_executable(publisher executables/publisher.cpp)
ament_target_dependencies(
  publisher
  "rclcpp"
  "std_msgs"
)

#ORIGINAL SUBSCRIBER
add_executable(subscriber executables/subscriber.cpp)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  subscriber
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)


# ##############################################################################
# Install ##
# ##############################################################################

# Install Libraries
install(
  TARGETS moveit_servo_lib_cpp moveit_servo_lib_parameters
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_servo)

# Install Binaries
install(
  TARGETS demo_joint_jog servo_node hello_moveit publisher subscriber
  DESTINATION lib/${PROJECT_NAME})  # Updated to use PROJECT_NAME for consistency

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include/moveit_servo)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})  # Updated to use PROJECT_NAME for consistency
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})  # Updated to use PROJECT_NAME for consistency

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
