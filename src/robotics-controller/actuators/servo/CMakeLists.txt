# ==============================================================================
# Servo Controller Library
# ==============================================================================

add_library(servo_controller STATIC
    servo_controller.cpp
)

target_include_directories(servo_controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link GPIOD if available (for PWM)
if(GPIOD_FOUND)
    target_link_libraries(servo_controller gpiod)
    target_compile_definitions(servo_controller PRIVATE HAVE_GPIOD=1)
endif()

message(STATUS "Servo controller configured")
