# Motor Controller - Beginner's Guide

This document explains how to use the refactored `MotorController` class, which has been designed to be beginner-friendly and easy to understand.

## Overview

The `MotorController` class manages two DC motors (typically for a robot's left and right wheels). It uses:
- **PWM (Pulse Width Modulation)** to control motor speed
- **GPIO pins** to control motor direction
- **Error handling** to catch and report problems

## Key Features

✅ **Beginner-friendly API** - Simple, clear method names  
✅ **Comprehensive error handling** - Know exactly what went wrong  
✅ **Automatic resource management** - No memory leaks or cleanup worries  
✅ **Well-documented code** - Every function explained  
✅ **Input validation** - Speed values are automatically clamped to valid ranges  
✅ **Hardware abstraction** - Works with different pin configurations  

## Quick Start

### 1. Include the Header
```cpp
#include "motor_controller.hpp"
using namespace robotics;
```

### 2. Configure Your Hardware
```cpp
// Create configuration for your specific hardware setup
MotorControllerConfig config;

// Left motor pins
config.left_motor.pwm_pin = 0;   // PWM pin for speed control
config.left_motor.dir1_pin = 17; // Direction control pin 1
config.left_motor.dir2_pin = 27; // Direction control pin 2

// Right motor pins  
config.right_motor.pwm_pin = 1;   // PWM pin for speed control
config.right_motor.dir1_pin = 22; // Direction control pin 1
config.right_motor.dir2_pin = 23; // Direction control pin 2
```

### 3. Create and Initialize
```cpp
// Create motor controller with your configuration
MotorController motor_controller(config);

// Initialize the hardware
MotorError result = motor_controller.initialize();
if (result != MotorError::SUCCESS) {
    std::cerr << "Initialization failed: " 
              << MotorController::error_to_string(result) << std::endl;
    return -1;
}
```

### 4. Control the Motors
```cpp
// Move forward at 50% speed
motor_controller.set_speed(50.0, 50.0);

// Turn left (left motor slower)
motor_controller.set_speed(25.0, 75.0);

// Move backward at 30% speed
motor_controller.set_speed(-30.0, -30.0);

// Stop completely
motor_controller.stop();
```

## Understanding Motor Control

### Speed Values
- **Range**: -100.0 to +100.0
- **Positive values**: Motor moves forward
- **Negative values**: Motor moves backward  
- **Zero**: Motor stops
- **Values are automatically clamped** to the valid range

### Motor Directions
The controller uses two direction pins per motor:
- **Forward**: dir1=HIGH, dir2=LOW
- **Backward**: dir1=LOW, dir2=HIGH
- **Stop/Brake**: dir1=LOW, dir2=LOW

### PWM (Speed Control)
- PWM controls how much power goes to the motor
- Higher duty cycle = faster speed
- The controller automatically converts your percentage to the correct PWM values

## Error Handling

The motor controller uses an enum for clear error reporting:

```cpp
enum class MotorError {
    SUCCESS,           // Everything worked fine
    GPIO_INIT_FAILED,  // Could not access GPIO pins
    PWM_SETUP_FAILED,  // PWM configuration failed
    INVALID_SPEED,     // Speed value out of range
    NOT_INITIALIZED    // Must call initialize() first
};
```

Always check return values:
```cpp
MotorError result = motor_controller.set_speed(75.0, -50.0);
if (result != MotorError::SUCCESS) {
    std::cout << "Error: " << MotorController::error_to_string(result) << std::endl;
}
```

## Class Structure (For Learning)

The refactored motor controller is organized into clear, understandable parts:

### Main Class: `MotorController`
- **Public interface** - Methods you call
- **Configuration** - Hardware setup
- **Error handling** - Clear error reporting

### Helper Classes (Internal)
1. **`GPIOManager`** - Handles all GPIO operations
2. **`PWMController`** - Manages PWM for speed control  
3. **`SingleMotor`** - Controls one individual motor

This separation makes the code easier to understand and maintain.

## Common Usage Patterns

### Robot Movement
```cpp
// Move forward
motor_controller.set_speed(50.0, 50.0);

// Turn right (right motor slower)
motor_controller.set_speed(75.0, 25.0);

// Turn left (left motor slower) 
motor_controller.set_speed(25.0, 75.0);

// Spin in place (motors in opposite directions)
motor_controller.set_speed(50.0, -50.0);

// Stop
motor_controller.stop();
```

### Status Monitoring
```cpp
// Check if controller is ready
if (motor_controller.is_initialized()) {
    std::cout << "Controller is ready!" << std::endl;
}

// Get current speeds
double left = motor_controller.get_left_speed();
double right = motor_controller.get_right_speed();

// Get status string
std::string status = motor_controller.get_status();
std::cout << "Status: " << status << std::endl;
```

## Hardware Requirements

- **GPIO access** - Must run on a system with GPIO pins (like Raspberry Pi)
- **PWM support** - System must support PWM output
- **Motor driver** - You need a motor driver board (like L298N) between this controller and your motors
- **Power supply** - Adequate power for your motors

## Best Practices

1. **Always check return values** from motor controller methods
2. **Initialize once** at program startup
3. **Stop motors** before shutting down your program  
4. **Use appropriate speeds** - start with low values and increase gradually
5. **Test with one motor first** before connecting both

## Troubleshooting

### "GPIO initialization failed"
- Check if your program has permission to access GPIO
- Verify GPIO pins are not already in use
- Make sure you're running on hardware with GPIO support

### "PWM setup failed"  
- Check if PWM pins are available on your system
- Verify PWM is not already exported by another program
- Try different PWM pin numbers

### Motors don't move
- Check motor driver connections
- Verify power supply to motors
- Test with multimeter to confirm PWM output
- Check if motor driver is properly wired

## Example Files

- `motor_example.cpp` - Complete working example
- `motor_controller.hpp` - Header file with full documentation
- `motor_controller.cpp` - Implementation (study this to learn more)

## Next Steps

Once you're comfortable with basic motor control, you can:
1. Add speed ramping (gradual acceleration/deceleration)
2. Implement PID control for precise movements
3. Add encoders for position feedback
4. Create higher-level movement functions (move_forward, turn_degrees, etc.)

Happy coding! 🤖
