

add_executable(ex4_test_occupancy_grid
        test_occupancy_grid.cc
        )

target_link_libraries(ex4_test_occupancy_grid
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )


set_target_properties(ex4_test_occupancy_grid PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)