# ==============================================================================
# Button Sensor Library
# ==============================================================================

add_library(button_sensor STATIC
    button_sensor.cpp
)

target_include_directories(button_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available
if(GPIOD_FOUND)
    target_link_libraries(button_sensor gpiod)
    target_compile_definitions(button_sensor PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "Button sensor configured")
