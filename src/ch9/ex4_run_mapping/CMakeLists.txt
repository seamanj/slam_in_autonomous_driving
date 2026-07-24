add_executable(ex4_run_mapping
        run_mapping.cc
        )

target_link_libraries(ex4_run_mapping
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex4_run_mapping PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)