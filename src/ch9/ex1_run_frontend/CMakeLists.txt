add_executable(ex1_run_frontend
        run_frontend.cc
        )

target_link_libraries(ex1_run_frontend
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex1_run_frontend PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)