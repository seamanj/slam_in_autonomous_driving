

add_executable(ex3_test_2d_icp_likelihood
        test_2d_icp_likelihood.cc
        )

target_link_libraries(ex3_test_2d_icp_likelihood
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )


set_target_properties(ex3_test_2d_icp_likelihood PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)