# Robotics Controller - CMake Build System

This document describes the CMake build system for the Robotics Controller project.

## Overview

The CMake build system provides a unified configuration for building the Robotics Controller project with all its components, dependencies, and optional features.

## Benefits

1. **Unified Configuration** - Single CMake configuration handles all project aspects
2. **Simplified Build** - Straightforward build process with clear options
3. **Cross-Platform** - Works across different operating systems and architectures
4. **Dependency Management** - Automatic detection and configuration of dependencies
5. **Flexible Options** - Enable/disable features as needed

## File Structure

```text
src/
├── cmake/                              # CMake utility modules
│   ├── RoboticsConfig.cmake           # Common configuration utilities
│   ├── RoboticsDependencies.cmake     # Dependency management helpers
│   ├── RoboticsTesting.cmake          # Testing utilities
│   ├── RoboticsInstallation.cmake     # Installation helpers
│   └── RoboticsControllerConfig.cmake.in  # Package config template
├── CMakeLists.txt                     # Main CMake configuration
├── build.sh                          # Build script
└── robotics-controller/
    ├── CMakeLists.txt                 # Controller CMake configuration
    ├── actuators/                     # Actuator components
    ├── communication/                 # Communication modules
    ├── core/                          # Core functionality
    ├── interfaces/                    # Interface definitions
    └── sensors/                       # Sensor components
```

## Key Features

### Dependency Management

- **Automatic Detection** - Finds required libraries and tools automatically
- **OpenCV Integration** - Seamless OpenCV integration for computer vision
- **GPIO Support** - Hardware GPIO interface through libgpiod
- **Web Interface** - Optional web-based control interface
- **Cross-Platform** - Supports Linux, embedded systems, and development environments

### Build Configuration

- **Debug/Release Modes** - Optimized builds for development and production
- **Optional Components** - Enable/disable features as needed
- **Testing Support** - Integrated testing framework
- **Installation Setup** - Proper installation and packaging

## Usage

### Building the Project

Use the provided build script for the simplest experience:

```bash
# Basic build
./build.sh

# Debug build with tests
./build.sh --build-type Debug --tests

# Build without OpenCV
./build.sh --no-opencv

# Clean build with verbose output
./build.sh --clean --verbose
```

### Manual CMake Usage

```bash
# Configure
cmake -DCMAKE_BUILD_TYPE=Release -B build

# Build
cmake --build build --parallel $(nproc)

# Test
cd build && ctest --output-on-failure

# Install
cmake --install build
```

### Adding New Components

1. Create component directory with CMakeLists.txt
2. Add standard CMake configuration:

```cmake
# Add your component library
add_library(my_component STATIC
    my_component.cpp
    helper.cpp
)

# Set properties
set_target_properties(my_component PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(my_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies
target_link_libraries(my_component PUBLIC
    some_dependency
    Threads::Threads
)
```

## Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_ROBOTICS_CONTROLLER` | ON | Build the main robotics controller |
| `BUILD_WEB_INTERFACE` | ON | Build web interface components |
| `BUILD_TESTS` | OFF | Build test components |
| `ENABLE_DEBUG_LOGGING` | OFF | Enable debug logging |
| `BUILD_WITH_OPENCV` | ON | Build with OpenCV support |
| `BUILD_SHARED_LIBS` | OFF | Build shared libraries |

## Dependencies

### Required Dependencies

- **CMake 3.16+** - Build system
- **C++17 compatible compiler** - GCC 8+, Clang 7+, or MSVC 2019+
- **Threads** - Threading support

### Optional Dependencies

- **OpenCV 4.0+** - Computer vision functionality
- **libgpiod** - GPIO hardware interface (Linux)
- **nlohmann-json** - JSON parsing for configuration
- **Boost** - Additional utilities and libraries
- **Google Test** - Testing framework (if BUILD_TESTS=ON)

## Troubleshooting

### Common Issues

1. **Dependency not found** - Install missing dependencies or set CMAKE_PREFIX_PATH
2. **Compiler errors** - Ensure C++17 support is available
3. **OpenCV not found** - Install OpenCV development packages or disable with --no-opencv

### Debug Build

Enable verbose output to see detailed configuration:

```bash
./build.sh --verbose --build-type Debug
```

### Testing the Build System

```bash
# Test basic build
./build.sh --clean

# Test with all features
./build.sh --tests --debug-logging --clean

# Test minimal build
./build.sh --no-opencv --no-web --clean
```
