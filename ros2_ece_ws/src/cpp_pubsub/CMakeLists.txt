cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Publisher LIDAR
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

# Subscriber LIDAR
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs)

# Processor Ultrasonic
add_executable(ultrasonic_processor src/ultrasonic_processor.cpp)
ament_target_dependencies(ultrasonic_processor rclcpp std_msgs sensor_msgs)

# Buffer LIDAR (accumule les scans)
add_executable(lidar_buffer src/lidar_buffer_node.cpp)
ament_target_dependencies(lidar_buffer rclcpp std_msgs sensor_msgs)

# TF Broadcaster (publie la frame lidar_link)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros geometry_msgs)

# Install executables
install(TARGETS
  tf_broadcaster
  talker
  listener
  ultrasonic_processor
  lidar_buffer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
