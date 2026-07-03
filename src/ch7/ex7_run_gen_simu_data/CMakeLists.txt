

add_executable(ex7_run_gen_simu_data
        run_gen_simu_data.cc
        )

target_link_libraries(ex7_run_gen_simu_data
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )



set_target_properties(ex7_run_gen_simu_data PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch7)