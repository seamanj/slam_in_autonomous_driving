# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# ========== 强制使用系统 OpenCV（支持 GUI） ==========
# 优先使用系统 OpenCV，而不是 /usr/local 下的自定义版本
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4" CACHE PATH "Path to system OpenCV" FORCE)
# =====================================================

# ========== 查找基础依赖 ==========
find_package(angles REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)

# ===== 添加 GLOG 编译定义（解决新版本 GLOG 的包含问题） =====
add_definitions(-DGLOG_USE_GLOG_EXPORT)
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)
# ============================================================

find_package(g2o REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found, enabling parallelization")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(WARNING "OpenMP not found, performance may be affected")
endif()

# ========== ROS 2 依赖 ==========
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)      # 添加
find_package(rcl_interfaces REQUIRED)
# ========== rosbag2 ==========
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# ========== velodyne_msgs ==========
# 查找 velodyne_msgs（如果还没构建）
# find_package(velodyne_msgs REQUIRED)
#find_package(velodyne_driver REQUIRED)    # 可选

# ========== livox_ros_driver ==========
# 查找 livox_ros_driver（如果还没构建）
# find_package(livox_ros_driver REQUIRED)

# ========== 添加包含目录 ==========
include_directories(${angles_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Glog_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
if(NOT g2o_INCLUDE_DIRS)
    set(g2o_INCLUDE_DIRS "/usr/local/include")
    message(STATUS "Manually set g2o_INCLUDE_DIRS: ${g2o_INCLUDE_DIRS}")
endif()
include_directories(${g2o_INCLUDE_DIRS})
message(STATUS "g2o_INCLUDE_DIRS: ${g2o_INCLUDE_DIRS}")
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${rcl_interfaces_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})    # 添加
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${yaml-cpp_INCLUDE_DIRS})
include_directories(${rosbag2_cpp_INCLUDE_DIRS})
include_directories(${rosbag2_storage_INCLUDE_DIRS})
include_directories(${velodyne_msgs_INCLUDE_DIRS})    # 添加
include_directories(${livox_ros_driver_INCLUDE_DIRS}) # 添加

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# 其他 thirdparty
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/velodyne/include)

# ========== 一起构建消息包 ==========
# 先构建 monitor_msgs
# add_subdirectory(src/common/msg/monitor_msgs)

# 手动设置 monitor_msgs_DIR，让 velodyne_msgs 能找到
set(monitor_msgs_DIR ${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/monitor_msgs)

# 然后构建 velodyne_msgs
# add_subdirectory(src/common/msg/velodyne_msgs)

# livox_ros_driver
# add_subdirectory(thirdparty/livox_ros_driver)

# 添加生成的头文件路径
# include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/monitor_msgs/rosidl_generator_cpp)
# include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/velodyne_msgs/rosidl_generator_cpp)
# include_directories(${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp)

# ========== 添加 install/include 全局路径 ==========
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/install/include)


# Ubuntu 20.04+ / 22.04 使用系统 TBB
find_package(TBB REQUIRED)
#================ UI ==================
set(ui_libs
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#================ SLAM Core ==================
set(slam_core_libs
    g2o::core
    g2o::stuff
    g2o_solver_cholmod

    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}

    ${yaml-cpp_LIBRARIES}
    yaml-cpp

    TBB::tbb
    ${OpenMP_CXX_LIBRARIES}

    glog
    gflags
)

#================ ROS2 ==================
set(ros2_libs
    ${rclcpp_LIBRARIES}
    ${rclpy_LIBRARIES}
    ${rcl_interfaces_LIBRARIES}

    ${sensor_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}

    ${rosbag2_cpp_LIBRARIES}
    ${rosbag2_storage_LIBRARIES}

    ${pcl_ros_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
)
#================ Test ==================
set(test_libs
    GTest::gtest
    GTest::gtest_main
)
#================ Compatibility ==================
set(third_party_libs
    ${slam_core_libs}
)
