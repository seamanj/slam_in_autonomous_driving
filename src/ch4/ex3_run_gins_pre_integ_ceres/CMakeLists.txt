


add_executable(ex3_run_gins_pre_integ_ceres
        run_gins_pre_integ_ceres.cc
        gins_pre_integ_ceres.cc
        ${PROJECT_SOURCE_DIR}/src/ch4/imu_preintegration.cc)

target_link_libraries(ex3_run_gins_pre_integ_ceres
        ${PROJECT_NAME}.ch3
        Ceres::ceres)

set_target_properties(ex3_run_gins_pre_integ_ceres PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch4)