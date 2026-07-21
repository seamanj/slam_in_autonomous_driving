add_library(${PROJECT_NAME}.ch8
        lio-iekf/lio_iekf.cc
        lio-preinteg/lio_preinteg.cc
        )

target_link_libraries(${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(ex1_test_lio_iekf test/ex1_test_lio_iekf.cc)
target_link_libraries(ex1_test_lio_iekf
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch4
        )
set_target_properties(ex1_test_lio_iekf PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch8)


add_executable(ex2_test_lio_preinteg test/ex2_test_lio_preinteg.cc)
target_link_libraries(ex2_test_lio_preinteg
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch4
        )
set_target_properties(ex2_test_lio_preinteg PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch8)
