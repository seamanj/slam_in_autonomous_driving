

add_executable(ex4_test_feature_extraction
        test_feature_extraction.cc
        )

target_link_libraries(ex4_test_feature_extraction
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.ros2
        ${third_party_libs}
        
        )



set_target_properties(ex4_test_feature_extraction PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)