add_library(${PROJECT_NAME}.ros2
    rosbag_io.cc
)

target_link_libraries(${PROJECT_NAME}.ros2
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.io
    ${PROJECT_NAME}.tools

    ${slam_core_libs}
    ${ros2_libs}
)