# ==============================================================================
# IMU Sensor Library
# ==============================================================================

add_library(imu_sensor STATIC
    imu_sensor.cpp
)

target_include_directories(imu_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available (for SPI)
if(GPIOD_FOUND)
    target_link_libraries(imu_sensor gpiod)
    target_compile_definitions(imu_sensor PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "IMU sensor configured")
