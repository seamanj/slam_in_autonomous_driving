
add_executable(ex1_point_cloud_load_and_vis point_cloud_load_and_vis.cc)
target_link_libraries(ex1_point_cloud_load_and_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags
        )


set_target_properties(ex1_point_cloud_load_and_vis PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch5)