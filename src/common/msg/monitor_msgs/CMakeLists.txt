cmake_minimum_required(VERSION 3.8)
project(monitor_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/FaultInfo.msg"
  "msg/FaultVec.msg"
) 

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


# 这两行必须存在，才会生成 cmake 配置文件
ament_export_dependencies(rosidl_default_runtime)
# 导出包
ament_package()