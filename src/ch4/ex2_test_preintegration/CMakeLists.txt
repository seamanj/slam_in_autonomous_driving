

ADD_EXECUTABLE(ex2_test_preintegration test_preintegration.cc)
ADD_TEST(ex2_test_preintegration ex2_test_preintegration)

target_link_libraries(ex2_test_preintegration
         ${PROJECT_NAME}.ch4
         ${PROJECT_NAME}.ch3
         ${PROJECT_NAME}.common
         ${slam_core_libs}
         ${test_libs}
        )
set_target_properties(ex2_test_preintegration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch4)