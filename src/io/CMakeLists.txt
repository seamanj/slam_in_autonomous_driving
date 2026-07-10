add_library(${PROJECT_NAME}.io
    txt_io.cc
)

target_link_libraries(${PROJECT_NAME}.io
    ${PROJECT_NAME}.common
)