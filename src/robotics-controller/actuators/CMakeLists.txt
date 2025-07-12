# ==============================================================================
# Actuator Components
# ==============================================================================

# Build individual actuator libraries
add_subdirectory(motor)
add_subdirectory(servo)

# Actuator Manager Library
add_library(actuator_manager STATIC
    actuator_manager.cpp
)

target_include_directories(actuator_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link actuator manager to all individual actuator libraries
target_link_libraries(actuator_manager
    motor_controller
    servo_controller
)

message(STATUS "Actuator components configured")
