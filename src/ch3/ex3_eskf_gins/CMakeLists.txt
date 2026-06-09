
add_executable(ex3_run_eskf_gins run_eskf_gins.cc)
set_target_properties(ex3_run_eskf_gins PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch3
)

target_link_libraries(ex3_run_eskf_gins
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )