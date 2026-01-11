cmake_minimum_required(VERSION 3.12)
project(agv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 添加 external 子目录，里面定义了 eigen target 
add_subdirectory(external)

# 添加 lib 子目录，里面定义了 xCoreSDK / xMateModel 等 target
add_subdirectory(lib)

# 添加头文件路径
include_directories(include)

# 定义一个库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/agv_hardware_interface.cpp
)

# ROS2依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# 普通CMake target依赖
target_link_libraries(${PROJECT_NAME}
  eigen
  Rokae::Rokae
)

pluginlib_export_plugin_description_file(hardware_interface agv_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  eigen
  Rokae::Rokae
)

ament_package()
