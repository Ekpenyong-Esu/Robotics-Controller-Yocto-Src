# ==============================================================================
# Serial Communication Library
# ==============================================================================

add_library(serial_communication STATIC
    serial_communication.cpp
)

target_include_directories(serial_communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

message(STATUS "Serial communication configured")
