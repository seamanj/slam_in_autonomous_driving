

add_executable(ex2_test_2d_icp_s2s
        test_2d_icp_s2s.cc
        )

target_link_libraries(ex2_test_2d_icp_s2s
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        glog gflags
        ${PCL_LIBRARIES}
        )


set_target_properties(ex2_test_2d_icp_s2s PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)