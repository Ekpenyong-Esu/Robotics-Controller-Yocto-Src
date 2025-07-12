# ==============================================================================
# Web Server Library
# ==============================================================================

add_library(web_server STATIC
    web_server.cpp
)

target_include_directories(web_server PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

message(STATUS "Web server configured")
