add_executable(ex5_dump_map
        dump_map.cc
        )

target_link_libraries(ex5_dump_map
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex5_dump_map PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)