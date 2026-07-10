add_executable(ex2_process_gnss process_gnss.cc)
target_link_libraries(ex2_process_gnss
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )

set_target_properties(ex2_process_gnss PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch3
)


add_library(${PROJECT_NAME}.ch3
        ../static_imu_init.cc
        ../utm_convert.cc
        # ieskf/nav_state_manifold.cc
        # ieskf/ieskf.cc

        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
        )

target_link_libraries(${PROJECT_NAME}.ch3
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.io
        )