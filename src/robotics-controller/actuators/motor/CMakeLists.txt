# ==============================================================================
# Motor Controller Library
# ==============================================================================

add_library(motor_controller STATIC
    motor_controller.cpp
)

target_include_directories(motor_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available (for PWM)
if(GPIOD_FOUND)
    target_link_libraries(motor_controller gpiod)
    target_compile_definitions(motor_controller PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "Motor controller configured")
