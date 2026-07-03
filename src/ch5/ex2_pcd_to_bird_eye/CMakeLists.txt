




add_executable(ex2_pcd_to_bird_eye pcd_to_bird_eye.cc)
target_link_libraries(ex2_pcd_to_bird_eye
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        ${PROJECT_NAME}.common
        )


set_target_properties(ex2_pcd_to_bird_eye PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch5)