


add_library(${PROJECT_NAME}.ch5
        bfnn.cc
        kdtree.cc
        octo_tree.cc
        )
target_link_libraries(${PROJECT_NAME}.ch5
        tbb
        )



add_subdirectory(ex1_point_cloud_load_and_vis)
add_subdirectory(ex2_pcd_to_bird_eye)
add_subdirectory(ex3_scan_to_range_image)
add_subdirectory(ex4_linear_fitting)
add_subdirectory(ex5_test_nn)