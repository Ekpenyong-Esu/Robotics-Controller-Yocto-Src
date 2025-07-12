# ==============================================================================
# Core System Components
# ==============================================================================

# System Manager Library
add_library(system_manager STATIC
    system_manager.cpp
)

target_include_directories(system_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# State Machine Library
add_library(state_machine STATIC
    state_machine.cpp
)

target_include_directories(state_machine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

message(STATUS "Core components configured")
