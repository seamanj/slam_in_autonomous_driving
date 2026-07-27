add_library(${PROJECT_NAME}.ch10
        fusion.cc
        )

target_link_libraries(${PROJECT_NAME}.ch10
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch7
        ${third_party_libs}
        )

add_executable(run_fusion_offline run_fusion_offline.cc)
target_link_libraries(run_fusion_offline
        ${PROJECT_NAME}.ch10
        ${third_party_libs}
        )

set_target_properties(run_fusion_offline PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch10)