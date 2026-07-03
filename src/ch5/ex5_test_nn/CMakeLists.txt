
ADD_EXECUTABLE(ex5_test_nn test_nn.cc)
ADD_TEST(ex5_test_nn ex5_test_nn)

target_link_libraries(ex5_test_nn
        gtest pthread glog gflags ${PROJECT_NAME}.ch5 ${PROJECT_NAME}.common ${PCL_LIBRARIES} tbb
        )

set_target_properties(ex5_test_nn PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch5)