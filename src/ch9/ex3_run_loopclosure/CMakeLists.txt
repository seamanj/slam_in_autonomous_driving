add_executable(ex3_run_loopclosure
        run_loopclosure.cc
        )

target_link_libraries(ex3_run_loopclosure
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex3_run_loopclosure PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)