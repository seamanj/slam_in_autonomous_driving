add_library(${PROJECT_NAME}.ch6
        icp_2d.cc
        lidar_2d_utils.cc
        likelihood_field.cc
        occupancy_map.cc
        submap.cc
        mapping_2d.cc
        multi_resolution_likelihood_field.cc
        loop_closing.cc
        frame.cc
        )

target_link_libraries(${PROJECT_NAME}.ch6
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}.common
        )



add_subdirectory(ex1_test_2dlidar_io)
add_subdirectory(ex2_test_2d_icp_s2s)
add_subdirectory(ex3_test_2d_icp_likelihood)
add_subdirectory(ex4_test_occupancy_grid)
add_subdirectory(ex5_test_2d_mapping)
add_subdirectory(ex6_test_mr_matching)