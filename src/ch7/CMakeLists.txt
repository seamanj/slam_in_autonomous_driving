add_library(${PROJECT_NAME}.ch7
        icp_3d.cc
        ndt_3d.cc
        gen_simu_data.cc
        direct_ndt_lo.cc
        # incremental_ndt_lo.cc
        # ndt_inc.cc
        # loam-like/feature_extraction.cc
        # loam-like/loam_like_odom.cc

        # loosely_coupled_lio/loosely_lio.cc
        # loosely_coupled_lio/cloud_convert.cc
        # loosely_coupled_lio/measure_sync.cc
        )

target_link_libraries(${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )


add_subdirectory(ex1_test_icp)
add_subdirectory(ex2_test_ndt_lo)
# add_subdirectory(ex3_test_inc_ndt_lo)
# add_subdirectory(ex4_test_feature_extraction)
# add_subdirectory(ex5_test_loam_odom)
# add_subdirectory(ex6_test_loosely_lio)
# add_subdirectory(ex7_run_gen_simu_data)
