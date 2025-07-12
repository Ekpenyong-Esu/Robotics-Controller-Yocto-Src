# ==============================================================================
# Line Sensor Array Library
# ==============================================================================

add_library(line_sensor_array STATIC
    line_sensor_array.cpp
)

target_include_directories(line_sensor_array PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available (for ADC)
if(GPIOD_FOUND)
    target_link_libraries(line_sensor_array gpiod)
    target_compile_definitions(line_sensor_array PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "Line sensor array configured")
