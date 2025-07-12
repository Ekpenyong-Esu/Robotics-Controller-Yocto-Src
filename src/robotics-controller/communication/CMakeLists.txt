# ==============================================================================
# Communication Components
# ==============================================================================

# Build individual communication libraries
add_subdirectory(serial)
add_subdirectory(web)

# Communication Hub Library
add_library(communication_hub STATIC
    communication_hub.cpp
)

target_include_directories(communication_hub PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link communication hub to all individual communication libraries
target_link_libraries(communication_hub
    serial_communication
    web_server
)

message(STATUS "Communication components configured")
