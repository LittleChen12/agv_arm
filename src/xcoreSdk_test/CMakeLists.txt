cmake_minimum_required(VERSION 3.12)
project(xcoreSdk_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 添加 external 子目录，里面定义了 eigen target 
add_subdirectory(external)

# 添加 lib 子目录，里面定义了 xCoreSDK / xMateModel 等 target
add_subdirectory(lib)

# 添加头文件路径
include_directories(
  include
)

# 定义一个可执行文件
add_executable(test_node src/test_node.cpp)

# 包含头文件路径
target_include_directories(test_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/xcoreSdk_test> 
  $<INSTALL_INTERFACE:include> 
)

# 链接库和依赖
# 注意：这里不再使用 xCoreSDK/xMateModel，而是用 lib/CMakeLists.txt 提供的别名 Rokae::Rokae
target_link_libraries(test_node
  Rokae::Rokae
  rclcpp::rclcpp
  eigen
)

# 安装规则
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
