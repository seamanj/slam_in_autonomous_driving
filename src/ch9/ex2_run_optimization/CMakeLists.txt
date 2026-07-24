add_executable(ex2_run_optimization
        run_optimization.cc
        )

target_link_libraries(ex2_run_optimization
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex2_run_optimization PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)