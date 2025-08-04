# Servo Controller - Beginner's Guide

This document explains how to use the refactored `ServoController` class, which has been designed to be beginner-friendly and easy to understand.

## Overview

The `ServoController` class manages hobby servo motors commonly used in robotics for precise angular positioning. It uses:
- **PWM (Pulse Width Modulation)** to control servo position
- **Hardware abstraction** for different servo types and pin configurations
- **Error handling** to catch and report problems
- **Smooth movement** options for gradual positioning

## Key Features

✅ **Beginner-friendly API** - Simple, clear method names  
✅ **Comprehensive error handling** - Know exactly what went wrong  
✅ **Automatic resource management** - No memory leaks or cleanup worries  
✅ **Well-documented code** - Every function explained  
✅ **Input validation** - Angle values are automatically clamped to valid ranges  
✅ **Hardware abstraction** - Works with different servo types and configurations  
✅ **Smooth movement** - Optional gradual movement for better control  
✅ **Position tracking** - Always know current and target positions  

## Quick Start

### 1. Include the Header
```cpp
#include "servo_controller.hpp"
using namespace robotics;
```

### 2. Configure Your Hardware (Optional)
```cpp
// Create configuration for your specific servo setup
ServoConfig config;
config.pwm_pin = 18;                    // PWM pin for servo control
config.min_pulse_width_ms = 0.5;        // Minimum pulse width (custom servo)
config.max_pulse_width_ms = 2.5;        // Maximum pulse width (custom servo)
config.frequency_hz = 50.0;             // Standard 50Hz frequency

// Or use defaults for standard servos:
// ServoConfig config; // Uses pin 18, 1-2ms pulses, 50Hz
```

### 3. Create and Initialize
```cpp
// Create servo controller with your configuration
ServoController servo_controller(config);

// Initialize the hardware
ServoError result = servo_controller.initialize();
if (result != ServoError::SUCCESS) {
    std::cerr << "Initialization failed: " 
              << ServoController::error_to_string(result) << std::endl;
    return -1;
}
```

### 4. Control the Servo
```cpp
// Move to center position (90 degrees)
servo_controller.center();

// Move to specific angles
servo_controller.set_angle(0.0);    // Minimum position
servo_controller.set_angle(180.0);  // Maximum position
servo_controller.set_angle(45.0);   // Custom position

// Smooth movement (gradual positioning)
servo_controller.set_angle_smooth(135.0, 0.2); // Move slowly to 135°

// In your main loop for smooth movement:
while (servo_controller.get_current_angle() != servo_controller.get_target_angle()) {
    servo_controller.update(); // Update position gradually
    usleep(10000); // Small delay (10ms)
}
```

## Understanding Servo Control

### How Servos Work
- **PWM Signal**: Servos use 50Hz PWM signals (20ms period)
- **Pulse Width**: The width of the pulse determines position:
  - 1ms pulse = 0° (minimum position)
  - 1.5ms pulse = 90° (center position) 
  - 2ms pulse = 180° (maximum position)
- **Continuous Signal**: Servos need a continuous PWM signal to maintain position

### Angle Values
- **Range**: 0.0 to 180.0 degrees
- **0°**: Minimum position (typically full counter-clockwise)
- **90°**: Center position
- **180°**: Maximum position (typically full clockwise)
- **Values are automatically clamped** to the valid range

### Movement Types

#### Immediate Movement
```cpp
servo.set_angle(90.0); // Servo moves immediately to 90°
```

#### Smooth Movement
```cpp
servo.set_angle_smooth(90.0, 0.1); // Gradual movement, 0.1 = slow
// Must call servo.update() regularly in your main loop
```

## Error Handling

The servo controller uses an enum for clear error reporting:

```cpp
enum class ServoError {
    SUCCESS,           // Everything worked fine
    GPIO_INIT_FAILED,  // Could not access GPIO system
    PWM_SETUP_FAILED,  // PWM configuration failed
    INVALID_ANGLE,     // Angle outside valid range
    NOT_INITIALIZED,   // Controller not initialized
    HARDWARE_ERROR     // General hardware problem
};
```

### Checking for Errors
```cpp
ServoError result = servo.set_angle(90.0);
if (result != ServoError::SUCCESS) {
    std::cout << "Error: " << ServoController::error_to_string(result) << std::endl;
}
```

## Complete Example

```cpp
#include "servo_controller.hpp"
#include <iostream>
#include <unistd.h>

using namespace robotics;

int main() {
    // Create servo controller with default configuration
    ServoController servo;
    
    // Initialize
    if (servo.initialize() != ServoError::SUCCESS) {
        std::cerr << "Failed to initialize servo!" << std::endl;
        return -1;
    }
    
    std::cout << "Servo initialized successfully!" << std::endl;
    
    // Move through different positions
    std::cout << "Moving to center..." << std::endl;
    servo.center();
    sleep(1);
    
    std::cout << "Moving to 0 degrees..." << std::endl;
    servo.set_angle(0.0);
    sleep(1);
    
    std::cout << "Moving to 180 degrees..." << std::endl;
    servo.set_angle(180.0);
    sleep(1);
    
    // Demonstrate smooth movement
    std::cout << "Smooth movement to 45 degrees..." << std::endl;
    servo.set_angle_smooth(45.0, 0.2);
    
    // Update loop for smooth movement
    while (std::abs(servo.get_current_angle() - servo.get_target_angle()) > 1.0) {
        servo.update();
        std::cout << "Current: " << servo.get_current_angle() 
                  << "°, Status: " << servo.get_status() << std::endl;
        usleep(50000); // 50ms delay
    }
    
    std::cout << "Movement complete!" << std::endl;
    
    // Shutdown automatically happens in destructor
    return 0;
}
```

## Configuration Options

### ServoConfig Structure
```cpp
struct ServoConfig {
    int pwm_pin;               // PWM pin number
    double min_pulse_width_ms; // Minimum pulse width in milliseconds
    double max_pulse_width_ms; // Maximum pulse width in milliseconds
    double frequency_hz;       // PWM frequency in Hz
};
```

### Common Servo Types

#### Standard Hobby Servo
```cpp
ServoConfig standard_servo;
standard_servo.pwm_pin = 18;
standard_servo.min_pulse_width_ms = 1.0;
standard_servo.max_pulse_width_ms = 2.0;
standard_servo.frequency_hz = 50.0;
```

#### High-Precision Servo
```cpp
ServoConfig precision_servo;
precision_servo.pwm_pin = 18;
precision_servo.min_pulse_width_ms = 0.5;
precision_servo.max_pulse_width_ms = 2.5;
precision_servo.frequency_hz = 50.0;
```

#### Digital Servo (Higher Frequency)
```cpp
ServoConfig digital_servo;
digital_servo.pwm_pin = 18;
digital_servo.min_pulse_width_ms = 1.0;
digital_servo.max_pulse_width_ms = 2.0;
digital_servo.frequency_hz = 333.0; // Some digital servos use higher frequencies
```

## Status Monitoring

```cpp
// Check initialization status
if (servo.is_initialized()) {
    std::cout << "Servo is ready!" << std::endl;
}

// Get current position
double current = servo.get_current_angle();
double target = servo.get_target_angle();

// Get human-readable status
std::string status = servo.get_status();
std::cout << "Status: " << status << std::endl;
```

## Best Practices

### 1. Always Initialize First
```cpp
if (servo.initialize() != ServoError::SUCCESS) {
    // Handle error before using servo
    return -1;
}
```

### 2. Check Return Values
```cpp
ServoError result = servo.set_angle(90.0);
if (result != ServoError::SUCCESS) {
    std::cout << "Failed: " << ServoController::error_to_string(result) << std::endl;
}
```

### 3. Use Smooth Movement for Better Control
```cpp
// Instead of immediate jumps:
servo.set_angle(180.0);

// Use smooth movement:
servo.set_angle_smooth(180.0, 0.3);
while (servo.get_current_angle() != servo.get_target_angle()) {
    servo.update();
    usleep(10000);
}
```

### 4. Proper Shutdown
```cpp
// Explicit shutdown (optional - destructor does this automatically)
servo.shutdown();
```

## Troubleshooting

### Common Issues

#### "PWM setup failed"
- Check if you have permission to access `/sys/class/pwm/`
- Verify the PWM pin number is correct for your hardware
- Make sure PWM is not already in use by another process

#### "GPIO initialization failed"
- Run with proper permissions (might need sudo)
- Check if GPIO library is installed
- Verify hardware connections

#### Servo doesn't move
- Check power supply to servo (servos need external power)
- Verify PWM signal reaches the servo
- Confirm servo is not damaged

#### Erratic movement
- Check power supply stability
- Verify timing parameters match your servo's specifications
- Reduce movement speed for better control

## Hardware Setup

### Typical Connections
```
Raspberry Pi  →  Servo Motor
GPIO 18 (PWM) →  Signal (Yellow/White wire)
5V/3.3V       →  Power (Red wire)
Ground        →  Ground (Black/Brown wire)
```

### Important Notes
- Most servos require external power (5V, high current)
- Signal wire typically needs 3.3V or 5V logic levels
- Always connect grounds together
- Use proper gauge wires for power connections

## Advanced Usage

### Multiple Servos
```cpp
// Configure different servos
ServoConfig servo1_config;
servo1_config.pwm_pin = 18;

ServoConfig servo2_config;
servo2_config.pwm_pin = 19;

// Create multiple controllers
ServoController servo1(servo1_config);
ServoController servo2(servo2_config);

// Initialize both
servo1.initialize();
servo2.initialize();

// Control independently
servo1.set_angle(45.0);
servo2.set_angle(135.0);
```

### Custom Movement Patterns
```cpp
// Sweep back and forth
for (int i = 0; i < 5; i++) {
    servo.set_angle_smooth(0.0, 0.5);
    while (servo.get_current_angle() > 5.0) {
        servo.update();
        usleep(10000);
    }
    
    servo.set_angle_smooth(180.0, 0.5);
    while (servo.get_current_angle() < 175.0) {
        servo.update();
        usleep(10000);
    }
}
```

This refactored servo controller provides a clean, beginner-friendly interface while maintaining the flexibility needed for advanced robotics applications.
