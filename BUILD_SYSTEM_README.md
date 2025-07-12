# Robotics Controller - Modular CMake Build System

This document explains the new modular CMake build system for the Robotics Controller project.

## Project Structure

```
src/
├── CMakeLists.txt                 # Parent CMake file
├── build.sh                       # Convenience build script
├── robotics-controller/
│   ├── CMakeLists-modular.txt     # New modular CMake config
│   ├── CMakeLists.txt             # Original monolithic CMake config
│   ├── core/
│   │   └── CMakeLists.txt         # Core system components
│   ├── sensors/
│   │   ├── CMakeLists.txt         # Sensor manager + all sensors
│   │   ├── button/
│   │   │   └── CMakeLists.txt     # Button sensor library
│   │   ├── distance/
│   │   │   └── CMakeLists.txt     # Distance sensor library
│   │   ├── gps/
│   │   │   └── CMakeLists.txt     # GPS sensor library
│   │   ├── imu/
│   │   │   └── CMakeLists.txt     # IMU sensor library
│   │   ├── ir/
│   │   │   └── CMakeLists.txt     # IR sensor library
│   │   └── line/
│   │       └── CMakeLists.txt     # Line sensor array library
│   ├── actuators/
│   │   ├── CMakeLists.txt         # Actuator manager + all actuators
│   │   ├── motor/
│   │   │   └── CMakeLists.txt     # Motor controller library
│   │   └── servo/
│   │       └── CMakeLists.txt     # Servo controller library
│   ├── communication/
│   │   ├── CMakeLists.txt         # Communication hub + all modules
│   │   ├── serial/
│   │   │   └── CMakeLists.txt     # Serial communication library
│   │   └── web/
│   │       └── CMakeLists.txt     # Web server library
│   ├── navigation/
│   │   └── CMakeLists.txt         # Navigation engine library
│   └── vision/
│       └── CMakeLists.txt         # Vision processor library
├── web-interface/
│   └── CMakeLists.txt             # Web interface build support
└── config/
    └── (configuration files)
```

## Build System Features

### Modular Architecture
- Each sensor/component has its own CMakeLists.txt
- Individual libraries can be built and tested separately
- Hierarchical dependency management
- Clean separation of concerns

### Build Options
- **BUILD_INDIVIDUAL_TARGETS**: Build individual component libraries
- **BUILD_VISION_SUPPORT**: Enable/disable OpenCV vision support
- **BUILD_TESTS**: Build unit tests for all components
- **ENABLE_DEBUG_LOGGING**: Enable debug output across all components

### Multiple Build Methods

#### Method 1: Using the Build Script (Recommended)
```bash
# Build everything (default)
./src/build.sh

# Clean build in debug mode
./src/build.sh --clean --debug

# Build with tests enabled
./src/build.sh --tests

# Build specific target
./src/build.sh robotics-controller

# Show help
./src/build.sh --help
```

#### Method 2: Direct CMake (from src directory)
```bash
cd src
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### Method 3: Using Individual Components
```bash
cd src/robotics-controller
mkdir build && cd build

# Use the modular CMake file
cmake -f ../CMakeLists-modular.txt ..
make -j$(nproc)

# Or build specific components
make motor_controller
make sensor_manager
make vision_processor
```

## Available Targets

### Main Targets
- **robotics-controller-exe**: Main executable
- **robotics-controller-lib**: Main library with all components

### Individual Component Libraries
- **Core**: `system_manager`, `state_machine`
- **Sensors**: `sensor_manager`, `button_sensor`, `distance_sensor`, `gps_sensor`, `imu_sensor`, `ir_sensor`, `line_sensor_array`
- **Actuators**: `actuator_manager`, `motor_controller`, `servo_controller`
- **Communication**: `communication_hub`, `serial_communication`, `web_server`
- **Navigation**: `navigation_engine`
- **Vision**: `vision_processor`

### Development Targets
- **test-motor-controller**: Motor controller test executable
- **web-install**: Install web interface dependencies
- **web-build**: Build web interface
- **docs**: Generate documentation (if Doxygen available)
- **format**: Format code (if clang-format available)
- **analyze**: Static analysis (if cppcheck available)

## Testing Individual Components

```bash
# Build and test specific components
cd src/build
make button_sensor && ./test-button-sensor
make motor_controller && ./test-motor-controller

# Run all tests
./src/build.sh --tests
```

## Development Workflow

### Adding a New Sensor
1. Create the sensor directory: `src/robotics-controller/sensors/newsensor/`
2. Add source files: `newsensor.cpp`, `newsensor.hpp`
3. Create `CMakeLists.txt` following the pattern of existing sensors
4. Add the subdirectory to `src/robotics-controller/sensors/CMakeLists.txt`
5. Update sensor manager to include the new sensor

### Adding a New Component Type
1. Create the component directory: `src/robotics-controller/newcomponent/`
2. Create component CMakeLists.txt
3. Add subdirectory to main modular CMakeLists.txt
4. Update the parent src/CMakeLists.txt if needed

## Migration from Original CMakeLists.txt

The original CMakeLists.txt is preserved for compatibility. To migrate:

1. **Immediate**: Use `CMakeLists-modular.txt` for new development
2. **Gradual**: Test individual components with the modular system
3. **Complete**: Replace `CMakeLists.txt` with `CMakeLists-modular.txt` when ready

## Benefits of Modular System

1. **Faster Development**: Build only changed components
2. **Better Testing**: Test individual components in isolation
3. **Cleaner Dependencies**: Clear dependency relationships
4. **Easier Debugging**: Isolate issues to specific components
5. **Parallel Development**: Multiple developers can work on different components
6. **Selective Features**: Enable/disable components as needed

## Troubleshooting

### Common Issues
- **Missing GPIOD**: Install with `sudo apt-get install libgpiod-dev`
- **OpenCV Build Fails**: Disable with `--no-vision` flag
- **Permission Errors**: Ensure build script is executable

### Debug Build
```bash
./src/build.sh --debug --verbose
```

### Clean Rebuild
```bash
./src/build.sh --clean
```

This modular system provides a robust foundation for developing and testing the robotics controller while maintaining flexibility and scalability.
