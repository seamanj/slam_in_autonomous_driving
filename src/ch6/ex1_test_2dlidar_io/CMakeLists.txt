

add_executable(ex1_test_2dlidar_io
        test_2dlidar_io.cc
        )

target_link_libraries(ex1_test_2dlidar_io
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        glog gflags
        ${PCL_LIBRARIES}
        ${third_party_libs}
        )


set_target_properties(ex1_test_2dlidar_io PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch6)