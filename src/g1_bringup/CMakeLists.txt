cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)

# StateBridge component (+ standalone `state_bridge` executable, used for profiling).
add_library(state_bridge_component SHARED src/state_bridge.cpp)
target_include_directories(state_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(state_bridge_component PUBLIC cxx_std_17)
ament_target_dependencies(state_bridge_component rclcpp rclcpp_components geometry_msgs nav_msgs sensor_msgs
  tf2_ros unitree_go unitree_hg)
rclcpp_components_register_node(state_bridge_component
  PLUGIN "g1_bringup::StateBridge" EXECUTABLE state_bridge)

# Feeds (receive, stamp) pairs through ClockOffset: lets scripts/test_clock_offset.py test the C++ code on bags.
add_executable(clock_offset_check src/clock_offset_check.cpp)
target_include_directories(clock_offset_check PRIVATE include)
target_compile_features(clock_offset_check PRIVATE cxx_std_17)

install(TARGETS state_bridge_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# Feeds bag IMU samples + clouds through deskew_rotation(): lets scripts/test_deskew.py test the C++ code on bags.
add_executable(deskew_check src/deskew_check.cpp)
target_include_directories(deskew_check PRIVATE include)
target_compile_features(deskew_check PRIVATE cxx_std_17)
ament_target_dependencies(deskew_check sensor_msgs)

install(TARGETS clock_offset_check deskew_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
