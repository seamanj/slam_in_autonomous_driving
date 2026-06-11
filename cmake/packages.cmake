# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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
find_package(g2o REQUIRED)
find_package(Ceres REQUIRED)

# ROS 2 依赖
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# rosbag2
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# ========== 添加包含目录 ==========
include_directories(${angles_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Glog_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${yaml-cpp_INCLUDE_DIRS})
include_directories(${rosbag2_cpp_INCLUDE_DIRS})
include_directories(${rosbag2_storage_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# 其他 thirdparty
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/velodyne/include)

# ========== 一起构建消息包 ==========
# 先构建 monitor_msgs
add_subdirectory(src/common/msg/monitor_msgs)

# 手动设置 monitor_msgs_DIR，让 velodyne_msgs 能找到
set(monitor_msgs_DIR ${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/monitor_msgs)

# 然后构建 velodyne_msgs
add_subdirectory(src/common/msg/velodyne_msgs)

# livox_ros_driver
add_subdirectory(thirdparty/livox_ros_driver)

# 添加生成的头文件路径
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/monitor_msgs/rosidl_generator_cpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/common/msg/velodyne_msgs/rosidl_generator_cpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp)
# =================================

# ========== 添加 install/include 全局路径 ==========
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/install/include)
# =================================================

# ========== TBB 配置 ==========
if(BUILD_WITH_UBUNTU1804)
    # Ubuntu 18.04 配置
    function(extract_file filename extract_dir)
        message(STATUS "Extract ${filename} to ${extract_dir} ...")
        set(temp_dir ${extract_dir})
        if(EXISTS ${temp_dir})
            file(REMOVE_RECURSE ${temp_dir})
        endif()
        file(MAKE_DIRECTORY ${temp_dir})
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${filename}
                WORKING_DIRECTORY ${temp_dir})
    endfunction()

    set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
    set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

    extract_file(${PROJECT_SOURCE_DIR}/thirdparty/tbb/2019_U8.tar.gz ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8)

    include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

    tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++17
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

    find_package(TBB REQUIRED)

    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
    link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)


    set(third_party_libs
            ${rclcpp_LIBRARIES}
            ${rclpy_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            ${g2o_libs}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${Pangolin_LIBRARIES}
            glog gflags
            ${yaml-cpp_LIBRARIES}
            yaml-cpp
            TBB::tbb
            )
else()
    # Ubuntu 20.04+ / 22.04 使用系统 TBB
    find_package(TBB REQUIRED)
    set(third_party_libs
            ${rclcpp_LIBRARIES}
            ${rclpy_LIBRARIES}
            ${sensor_msgs_LIBRARIES}
            ${std_msgs_LIBRARIES}
            # g2o（使用现代写法）
            g2o::core
            g2o::stuff
            # rosbag2 依赖（添加这几行）
            ${rosbag2_cpp_LIBRARIES}
            ${rosbag2_storage_LIBRARIES}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${Pangolin_LIBRARIES}
            glog gflags
            ${yaml-cpp_LIBRARIES}
            yaml-cpp
            TBB::tbb
            )
endif()