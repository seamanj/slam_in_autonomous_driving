# 1. 添加可执行文件
add_executable(motion motion.cc)

# 2. 设置目标属性（必须在 add_executable 之后）
set_target_properties(motion PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch2
)

# 3. 链接库
target_link_libraries(motion
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.tools
)


add_executable(ex3_motion ex3_motion.cc)

# 设置 ex3_motion 的输出目录
set_target_properties(ex3_motion PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/ch2
)

# 链接 ex3_motion 的库
target_link_libraries(ex3_motion
    ${PROJECT_NAME}.common
    ${PROJECT_NAME}.tools
)