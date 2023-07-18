
add_library(${PROJECT_NAME}.tools
        io_utils.cc
        )

target_link_libraries(${PROJECT_NAME}.tools
        ${third_party_libs}
        )

