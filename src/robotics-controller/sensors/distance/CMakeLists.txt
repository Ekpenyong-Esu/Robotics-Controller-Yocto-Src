# ==============================================================================
# Distance Sensor Library
# ==============================================================================

add_library(distance_sensor STATIC
    distance_sensor.cpp
)

target_include_directories(distance_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available (for I2C)
if(GPIOD_FOUND)
    target_link_libraries(distance_sensor gpiod)
    target_compile_definitions(distance_sensor PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "Distance sensor configured")
