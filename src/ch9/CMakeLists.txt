add_library(${PROJECT_NAME}.ch9
        frontend.cc
        keyframe.cc
        optimization.cc
        loopclosure.cc
        )

target_link_libraries(${PROJECT_NAME}.ch9
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch4
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )


add_subdirectory(ex1_run_frontend)
add_subdirectory(ex2_run_optimization)
add_subdirectory(ex3_run_loopclosure)
add_subdirectory(ex4_run_mapping)
add_subdirectory(ex5_dump_map)
add_subdirectory(ex6_split_map)

