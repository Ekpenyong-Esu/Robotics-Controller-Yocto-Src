# ==============================================================================
# Robotics Controller Application - Modular CMakeLists.txt
# ==============================================================================

cmake_minimum_required(VERSION 3.14)

# Build all component libraries first
add_subdirectory(core)
add_subdirectory(sensors)
add_subdirectory(actuators)
add_subdirectory(communication)
add_subdirectory(navigation)
add_subdirectory(vision)

# Create main robotics-controller executable
add_executable(robotics-controller-exe
    main.cpp
)

# Set target name
set_target_properties(robotics-controller-exe PROPERTIES OUTPUT_NAME robotics-controller)

# Include directories
target_include_directories(robotics-controller-exe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link all component libraries
target_link_libraries(robotics-controller-exe
    # Core libraries
    system_manager
    state_machine

    # Sensor libraries
    sensor_manager
    button_sensor
    distance_sensor
    gps_sensor
    imu_sensor
    ir_sensor
    line_sensor_array

    # Actuator libraries
    actuator_manager
    motor_controller
    servo_controller

    # Communication libraries
    communication_hub
    serial_communication
    web_server

    # Navigation and vision
    navigation_engine
    vision_processor

    # System libraries
    Threads::Threads
    
    # OpenCV (needed for vision processor headers)
    opencv
)

# Link GPIOD if available
if(GPIOD_FOUND)
    target_link_libraries(robotics-controller-exe gpiod)
    target_compile_definitions(robotics-controller-exe PRIVATE HAVE_GPIOD=1)
endif()

# Install executable
install(TARGETS robotics-controller-exe
    DESTINATION bin
)

message(STATUS "Robotics controller application configured")
