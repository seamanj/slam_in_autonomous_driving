cmake_minimum_required(VERSION 3.8)
project(velodyne_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ROS 2 中需要显式 find_package monitor_msgs
find_package(monitor_msgs REQUIRED)

set(msg_files
  "msg/VelodynePacket.msg"
  "msg/VelodyneScan.msg"
  "msg/VelodyneScanRaw.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs monitor_msgs builtin_interfaces
)


# 这两行必须存在，才会生成 cmake 配置文件
ament_export_dependencies(rosidl_default_runtime)
# 导出包
ament_package()