cmake_minimum_required(VERSION 3.8)
project(visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(visualizer src/visualizer.cpp)
  ament_target_dependencies(visualizer rclcpp 
  geometry_msgs
  crp_msgs
  foxglove_msgs
  visualization_msgs)

install(TARGETS
  visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
