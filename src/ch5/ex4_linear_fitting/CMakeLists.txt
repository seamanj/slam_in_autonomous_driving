
add_executable(ex4_linear_fitting linear_fitting.cc)
target_link_libraries(ex4_linear_fitting
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        )


set_target_properties(ex4_linear_fitting PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch5)