


add_executable(ex3_scan_to_range_image scan_to_range_image.cc)
target_link_libraries(ex3_scan_to_range_image
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        ${PROJECT_NAME}.common
        )



set_target_properties(ex3_scan_to_range_image PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch5)