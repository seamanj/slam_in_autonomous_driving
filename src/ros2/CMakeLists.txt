add_library(${PROJECT_NAME}.ros2
    rosbag_io.cc

    velodyne/packets_parser.cc
    velodyne/velodyne_convertor.cc

    livox/cloud_convert.cc
    livox/measure_sync.cc

)

target_link_libraries(${PROJECT_NAME}.ros2
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.io
    ${PROJECT_NAME}.tools

    ${slam_core_libs}
    ${ros2_libs}

    # Velodyne
    velodyne_msgs::velodyne_msgs__rosidl_generator_cpp
    velodyne_msgs::velodyne_msgs__rosidl_typesupport_cpp

    # Livox
    livox_ros_driver::livox_ros_driver__rosidl_generator_cpp
    livox_ros_driver::livox_ros_driver__rosidl_typesupport_cpp

)