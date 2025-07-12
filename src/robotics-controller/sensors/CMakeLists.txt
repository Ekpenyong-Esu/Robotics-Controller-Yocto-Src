# ==============================================================================
# Sensor Components
# ==============================================================================

# Build individual sensor libraries
add_subdirectory(button)
add_subdirectory(distance)
add_subdirectory(gps)
add_subdirectory(imu)
add_subdirectory(ir)
add_subdirectory(line)

# Sensor Manager Library
add_library(sensor_manager STATIC
    sensor_manager.cpp
)

target_include_directories(sensor_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link sensor manager to all individual sensor libraries
target_link_libraries(sensor_manager
    button_sensor
    distance_sensor
    gps_sensor
    imu_sensor
    ir_sensor
    line_sensor_array
)

message(STATUS "Sensor components configured")
