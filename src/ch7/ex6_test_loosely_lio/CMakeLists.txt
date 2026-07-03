

add_executable(ex6_test_loosely_lio
        test_loosely_lio.cc
        )

target_link_libraries(ex6_test_loosely_lio
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )



set_target_properties(ex6_test_loosely_lio PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)