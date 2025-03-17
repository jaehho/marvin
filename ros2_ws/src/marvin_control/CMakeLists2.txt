cmake_minimum_required(VERSION 3.8)
project(marvin_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)  

#SERVO POSE TRACKING
add_executable(pose_tracking src/pose_tracking.cpp)
# might be able to delete the thing below
target_include_directories(pose_tracking PUBLIC include)
ament_target_dependencies(
  pose_tracking
  "rclcpp"
  "trajectory_msgs"  
  "moveit_servo"
  "control_msgs"
  "moveit_msgs"
)

#TARGET JOINT TRAJECTORY PUBLISHER
add_executable(joint_trajectory_publisher src/joint_trajectory_publisher.cpp)
ament_target_dependencies(
  joint_trajectory_publisher  # Updated the target name
  "rclcpp"
  "std_msgs"
  "trajectory_msgs"  # Updated to include trajectory_msgs
)
  
#HELLO MOVEIT
add_executable(hello_moveit src/hello_moveit.cpp)
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
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(
  publisher
  "rclcpp"
  "std_msgs"
)

#ORIGINAL SUBSCRIBER
add_executable(subscriber src/subscriber.cpp)
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

install(
  TARGETS
    pose_tracking joint_trajectory_publisher hello_moveit publisher subscriber
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
