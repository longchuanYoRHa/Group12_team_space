cmake_minimum_required(VERSION 3.5)
project(custom_explore)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined.")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "eloquent")
  message(STATUS "Build for ROS2 eloquent")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DELOQUENT")
elseif("$ENV{ROS_DISTRO}" STREQUAL "dashing")
  message(STATUS "Build for ROS2 dashing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDASHING")
else()
  message(STATUS "Build for ROS2: " "$ENV{ROS_DISTRO}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  map_msgs
  visualization_msgs
  nav2_costmap_2d
)

include_directories(
  include
)

add_executable(custom_explore_node
  src/costmap_client.cpp
  src/frontier_search.cpp
  src/explore_wait.cpp
)

target_include_directories(custom_explore_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_explore_node ${DEPENDENCIES})

install(TARGETS custom_explore_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

