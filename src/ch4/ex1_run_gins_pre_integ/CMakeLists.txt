


add_executable(ex1_run_gins_pre_integ run_gins_pre_integ.cc)
target_link_libraries(ex1_run_gins_pre_integ
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch4
        ${g2o_libs}
        )
set_target_properties(ex1_run_gins_pre_integ PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch4)