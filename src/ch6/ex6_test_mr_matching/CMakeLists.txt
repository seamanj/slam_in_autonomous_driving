
add_executable(ex6_test_mr_matching
        test_mr_matching.cc
        )
target_link_libraries(ex6_test_mr_matching
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )


set_target_properties(ex6_test_mr_matching PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)