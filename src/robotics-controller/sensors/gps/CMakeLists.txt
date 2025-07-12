# ==============================================================================
# GPS Sensor Library
# ==============================================================================

add_library(gps_sensor STATIC
    gps_sensor.cpp
)

target_include_directories(gps_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

message(STATUS "GPS sensor configured")
