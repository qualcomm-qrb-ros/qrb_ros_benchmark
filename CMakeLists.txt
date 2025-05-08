cmake_minimum_required(VERSION 3.8)
project(qrb_ros_benchmark)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

include_directories("${PROJECT_SOURCE_DIR}/include")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add qrb_playback_node lib
ament_auto_add_library(qrb_playback_node SHARED
    src/qrb_playback_node.cpp)

rclcpp_components_register_nodes(qrb_playback_node "qrb_ros::benchmark::QrbPlaybackNode")
set(node_plugins "${node_plugins}qrb_ros::benchmark::QrbPlaybackNode;$<TARGET_FILE:qrb_playback_node>\n")

# add qrb_monitor_node lib
ament_auto_add_library(qrb_monitor_node SHARED
    src/qrb_monitor_node.cpp)

rclcpp_components_register_nodes(qrb_monitor_node "qrb_ros::benchmark::QrbMonitorNode")
set(node_plugins "${node_plugins}qrb_ros::benchmark::QrbMonitorNode;$<TARGET_FILE:qrb_monitor_node>\n")

install(TARGETS
qrb_playback_node
qrb_monitor_node
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

ament_auto_package()
