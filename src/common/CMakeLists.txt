
# # 查找已安装的消息包
# find_package(monitor_msgs REQUIRED)
# find_package(velodyne_msgs REQUIRED)


# # 添加消息包的包含目录（关键！）
# include_directories(${monitor_msgs_INCLUDE_DIRS})
# include_directories(${velodyne_msgs_INCLUDE_DIRS})

# # 打印实际添加的路径
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)



# 然后添加 common 库
add_library(${PROJECT_NAME}.common
        # io_utils.cc
        timer/timer.cc
        global_flags.cc
        g2o_types.cc
        point_cloud_utils.cc
        path_utils.cc
        )

target_link_libraries(${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        ${PCL_LIBRARIES}
        )