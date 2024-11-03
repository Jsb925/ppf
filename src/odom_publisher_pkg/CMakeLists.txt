cmake_minimum_required(VERSION 3.5)
project(odom_publisher_pkg)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Create executable
add_executable(odom_publisher_node src/odom_publisher_node.cpp)
add_executable(scan_to_occ_node src/scan_to_occ_node.cpp)

# Link against the libraries
ament_target_dependencies(odom_publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
)

ament_target_dependencies(scan_to_occ_node 
  sensor_msgs 
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  odom_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  scan_to_occ_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install the launch files if you have any (optional)
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_package()
