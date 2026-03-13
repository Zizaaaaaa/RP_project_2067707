cmake_minimum_required(VERSION 3.8)
project(dmap_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Trova dipendenze ROS2 e Eigen
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories per Eigen e quelle testate
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# definizione libreria core
add_library(dmap_core
  src/dmap.cpp
  # src/localizer.cpp (da aggiungere man mano)
  # src/planner.cpp
)
ament_target_dependencies(dmap_core
  Eigen3
)

# Nodo di Localizzazione
add_executable(localizer_node src/localizer_node.cpp)
target_link_libraries(localizer_node dmap_core)
ament_target_dependencies(localizer_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros
)

# Nodo di Planning
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node dmap_core)
ament_target_dependencies(planner_node
  rclcpp nav_msgs geometry_msgs
)

install(TARGETS
  dmap_core
  localizer_node
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()