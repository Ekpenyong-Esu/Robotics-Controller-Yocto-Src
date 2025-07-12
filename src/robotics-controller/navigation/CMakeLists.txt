# ==============================================================================
# Navigation Engine Library
# ==============================================================================

add_library(navigation_engine STATIC
    navigation_engine.cpp
)

target_include_directories(navigation_engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

message(STATUS "Navigation engine configured")
