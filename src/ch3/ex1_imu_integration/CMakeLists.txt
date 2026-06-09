add_executable(ex1_run_imu_integration
        run_imu_integration.cc
        )
set_target_properties(ex1_run_imu_integration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch3
)

target_link_libraries(ex1_run_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )
