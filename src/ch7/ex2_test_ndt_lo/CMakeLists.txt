

add_executable(ex2_test_ndt_lo
        test_ndt_lo.cc
        )

target_link_libraries(ex2_test_ndt_lo
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        )



set_target_properties(ex2_test_ndt_lo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)