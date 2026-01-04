cmake_minimum_required(VERSION 3.8)
project(Controller_For_Agv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(agv_controllers src/main.cpp)
ament_target_dependencies(agv_controllers rclcpp moveit_ros_planning_interface)

install(TARGETS
  agv_controllers
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
