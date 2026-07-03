

add_executable(ex1_test_icp
        test_icp.cc
        )

target_link_libraries(ex1_test_icp
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )



set_target_properties(ex1_test_icp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)