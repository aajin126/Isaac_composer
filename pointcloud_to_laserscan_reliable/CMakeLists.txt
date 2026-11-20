cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_laserscan_reliable)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Upstream projection and TF filtering; only QoS is configurable locally.
add_library(${PROJECT_NAME}_core SHARED
  third_party/src/pointcloud_to_laserscan_node.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE POINTCLOUD_TO_LASERSCAN_BUILDING_DLL)
target_include_directories(${PROJECT_NAME}_core PUBLIC third_party/include)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp rclcpp_components sensor_msgs
  message_filters tf2 tf2_ros tf2_sensor_msgs)

add_executable(pointcloud_to_laserscan_node src/main.cpp)
target_link_libraries(pointcloud_to_laserscan_node ${PROJECT_NAME}_core)
ament_target_dependencies(pointcloud_to_laserscan_node rclcpp)

install(TARGETS ${PROJECT_NAME}_core pointcloud_to_laserscan_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY third_party/ DESTINATION share/${PROJECT_NAME}/third_party)
ament_package()
