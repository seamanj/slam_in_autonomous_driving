

add_executable(ex5_test_loam_odom
        test_loam_odom.cc
        )

target_link_libraries(ex5_test_loam_odom
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )



set_target_properties(ex5_test_loam_odom PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)