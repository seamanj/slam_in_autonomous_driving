cmake_minimum_required(VERSION 3.8)
project(slam_in_auto_driving)

# 设置编译类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 修改这里：保留 -w 但添加必要的定义
# set(CMAKE_CXX_FLAGS "-w -DEIGEN_USE_STD_CSTRING -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT")
# 新版sophus中的SO3用的四元组, 所以需要禁用对齐
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


list(PREPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/build/install")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")

# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble" "${CMAKE_BINARY_DIR}/install")
# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_WITH_UBUNTU1804 OFF)

# 包含自定义的 packages.cmake（所有依赖和消息包都在这里）
include(cmake/packages.cmake)

# 添加包含目录
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})

# 添加子目录
add_subdirectory(src)