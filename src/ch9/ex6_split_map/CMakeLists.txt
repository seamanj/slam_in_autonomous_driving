add_executable(ex6_split_map
        split_map.cc
        )

target_link_libraries(ex6_split_map
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )


set_target_properties(ex6_split_map PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch9)