# ==============================================================================
# IR Sensor Library
# ==============================================================================

add_library(ir_sensor STATIC
    ir_sensor.cpp
)

target_include_directories(ir_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available
if(GPIOD_FOUND)
    target_link_libraries(ir_sensor gpiod)
    target_compile_definitions(ir_sensor PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "IR sensor configured")
