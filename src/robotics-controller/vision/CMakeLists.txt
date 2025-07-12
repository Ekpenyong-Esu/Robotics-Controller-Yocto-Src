# ==============================================================================
# Vision Processor Library
# ==============================================================================

add_library(vision_processor STATIC
    vision_processor.cpp
)

target_include_directories(vision_processor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Always link OpenCV - required for vision processing
target_link_libraries(vision_processor PUBLIC opencv)
target_compile_definitions(vision_processor PRIVATE HAVE_OPENCV=1)

message(STATUS "Vision processor configured with OpenCV")
