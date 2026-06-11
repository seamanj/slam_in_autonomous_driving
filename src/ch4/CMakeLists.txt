add_library(${PROJECT_NAME}.ch4
        gins_pre_integ.cc
        imu_preintegration.cc
        g2o_types.cc
        )
add_subdirectory(ex1_run_gins_pre_integ)
add_subdirectory(ex2_test_preintegration)
add_subdirectory(ex3_run_gins_pre_integ_ceres)