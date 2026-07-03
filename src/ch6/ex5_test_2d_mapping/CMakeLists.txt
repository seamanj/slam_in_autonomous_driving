

add_executable(ex5_test_2d_mapping
        test_2d_mapping.cc
        )

target_link_libraries(ex5_test_2d_mapping
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )


set_target_properties(ex5_test_2d_mapping PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)