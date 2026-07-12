add_library(${PROJECT_NAME}.ros2
    rosbag_io.cc

    velodyne/packets_parser.cc
    velodyne/velodyne_convertor.cc

)

target_link_libraries(${PROJECT_NAME}.ros2
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.io
    ${PROJECT_NAME}.tools

    ${slam_core_libs}
    ${ros2_libs}

    velodyne_msgs::velodyne_msgs__rosidl_generator_cpp
    velodyne_msgs::velodyne_msgs__rosidl_typesupport_cpp

)