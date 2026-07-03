

add_executable(ex3_test_inc_ndt_lo
        test_inc_ndt_lo.cc
        )

target_link_libraries(ex3_test_inc_ndt_lo
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )



set_target_properties(ex3_test_inc_ndt_lo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)