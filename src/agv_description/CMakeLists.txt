cmake_minimum_required(VERSION 3.8)
project(agv_description)

# 必要的 ament 工具
find_package(ament_cmake REQUIRED)

# 将资源安装到 share/<package>
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

