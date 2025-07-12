# Embedded Robotics Controller with Yocto Project

A lightweight embedded robotics controller implementation for BeagleBone Black using the Yocto Project to create a customized Linux system optimized for real-time robotics applications.

## Project Overview

This project demonstrates how to create an efficient robotics controller using Yocto Project's flexible build system to create a custom Linux distribution. The system is designed for line-following and obstacle-avoiding robots with comprehensive I/O interface utilization.

### Why Yocto Project?

- **Flexibility**: Highly customizable Linux distributions for specific needs
- **Reproducibility**: Consistent builds with BitBake and metadata layers
- **Scalability**: Easily add software components via layered architecture
- **Standardization**: Industry-standard build system for embedded Linux
- **Real-time Performance**: Support for real-time kernels and patches

## Hardware Requirements

### Primary Board

- **BeagleBone Black** (preferred for PRUs and analog inputs) or Raspberry Pi 4

### Peripherals and I/O Interfaces

| Interface | Component | Purpose |
|-----------|-----------|---------|
| **GPIO** | LEDs, Push buttons, IR sensors | Status indication, mode selection, line following |
| **I2C** | VL53L0X time-of-flight sensor | Obstacle avoidance |
| **SPI** | MPU-9250 IMU | Orientation and motion tracking |
| **PWM** | DC motors + L298N driver | Robot movement control |
| **UART** | NEO-6M GPS module | Outdoor navigation |
| **Analog** | QTR-8A line sensors (BBB only) | Line following |
| **USB** | Webcam | Computer vision with OpenCV |

### Storage

- **SD Card**: 8GB or larger for Yocto Project image

## Software Requirements

### Host System

- Ubuntu 20.04+ with at least 50GB free disk space
- Internet connection for package downloads

### Tools and Libraries

- Yocto Project (Langdale or newer release)
- BitBake build system
- Git and Python 3
- OpenCV for computer vision
- meta-robotics layer (custom)
- meta-openembedded for additional packages

## Project Features

### 🎯 Core Functionality

- **Custom Linux Image**: Tailored system with essential drivers
- **Multi-Interface I/O**: Comprehensive utilization of all board interfaces
- **Real-time Control**: Low-latency motor and sensor control
- **Computer Vision**: OpenCV-based obstacle detection
- **Remote Monitoring**: Web interface for control and status

### 🚀 Advanced Features

- **PRU Integration**: Real-time units for precise motor control (BeagleBone Black)
- **GPS Navigation**: Outdoor positioning and waypoint navigation
- **IMU Fusion**: Orientation tracking and stability control
- **Adaptive Algorithms**: Dynamic behavior based on sensor inputs

## Quick Start

### 1. Environment Setup

```bash
# Clone the repository
git clone <your-repo-url>
cd Robotics-Controller-Yocto

# Make scripts executable
chmod +x scripts/*.sh

# Install host dependencies
sudo apt update
sudo apt install -y gawk wget git diffstat unzip texinfo gcc build-essential chrpath socat cpio python3 python3-pip python3-pexpect xz-utils debianutils iputils-ping python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev xterm python3-subunit mesa-common-dev zstd liblz4-tool
```

### 2. Build the System

```bash
# Initialize Yocto build environment with default configuration
./scripts/build.sh

# Initialize with QEMU for virtual testing
./scripts/build.sh --qemu

# Change to the build directory and source the environment
cd build
source setup-environment

# Build the robotics controller image
bitbake robotics-controller-image
```

### 3. Testing with QEMU (Virtual Testing)

```bash
# Run the system in QEMU (after building with --qemu option)
./scripts/run.sh

# Run with networking enabled
./scripts/run.sh --network

# Run with graphical interface
./scripts/run.sh --graphics
```

### 4. Deploy to Hardware

```bash
# Flash to SD card (replace /dev/sdX with your SD card device)
sudo ./scripts/flash.sh /dev/sdX

# Run the system (if using QEMU for testing)
./scripts/run.sh
```

## Project Structure

```text
Robotics-Controller-Yocto/
├── README.md                    # This file
├── meta-robotics/               # Custom Yocto layer
│   ├── conf/                    # Layer configuration
│   │   ├── layer.conf          # Layer definition  
│   │   └── machine/            # Machine configurations
│   │       ├── beaglebone-robotics.conf
│   │       ├── rpi4-robotics.conf
│   │       └── qemu-robotics.conf
│   ├── recipes-core/           # Core system recipes
│   │   └── images/
│   │       └── robotics-controller-image.bb
│   ├── recipes-robotics/        # Robotics-specific recipes
│   │   └── robotics-controller/ # Main application recipe
│   │       ├── files/          # Source files for robotics application
│   │       └── robotics-controller_1.0.bb
│   └── recipes-kernel/         # Kernel modifications
│       └── linux/              # Linux kernel configuration
│           ├── linux-yocto-rt/ # Real-time kernel configs for BeagleBone
│           └── linux-yocto-rt_%.bbappend
├── scripts/                     # Build and utility scripts
│   ├── build.sh                # Initialize build environment
│   ├── clean.sh                # Clean build artifacts
│   ├── save-config.sh          # Save current configuration
│   ├── flash.sh                # Flash image to SD card
│   └── run.sh                  # Run system (QEMU/hardware)
├── build/                       # Build directory (generated)
│   ├── conf/                   # Build configuration
│   │   ├── local.conf          # Local build settings
│   │   └── bblayers.conf       # Layer configuration
│   └── tmp/                    # Build artifacts
├── src/                        # Source code
│   ├── robotics-controller/    # Main C++ application
│   ├── drivers/                # Custom drivers
│   └── web-interface/          # Web control interface
├── docs/                       # Documentation
│   ├── hardware-setup.md       # Hardware connection guide
│   └── troubleshooting.md
└── tests/                      # Test scripts and utilities
```

## Build Scripts Usage

### Build Script (`./scripts/build.sh`)

```bash
# Initialize build environment
source ./scripts/build.sh

# Build robotics image
bitbake robotics-controller-image

# Build specific package
bitbake robotics-controller

# Build SDK for application development
bitbake robotics-controller-image -c populate_sdk
```

### Configuration Management

```bash
# Save current layer configuration
./scripts/save-config.sh

# Clean build directory
./scripts/clean.sh

# Flash to SD card
./scripts/flash.sh /dev/sdX
```

## Hardware Setup Guide

### 1. BeagleBone Black Connections

#### Power and Basic Setup

- Connect 5V power supply
- Insert SD card with flashed image
- Connect FTDI cable to UART0 for console access

#### Sensor Connections

##### I2C (VL53L0X Distance Sensor)

```text
P9.19 → SCL
P9.20 → SDA
P9.3  → VCC (3.3V)
P9.1  → GND
```

##### SPI (MPU-9250 IMU)

```text
P9.22 → SCLK
P9.18 → MOSI
P9.21 → MISO
P9.17 → CS
P9.3  → VCC (3.3V)
P9.1  → GND
```

##### PWM (Motor Control)

```text
P9.14 → Motor Driver PWM1
P9.16 → Motor Driver PWM2
```

##### GPIO (LEDs and Sensors)

```text
P8.13 → Status LED
P8.15 → IR Sensor 1
P8.17 → IR Sensor 2
P8.19 → Button Input
```

##### UART (GPS Module)

```text
P9.24 → GPS RX
P9.26 → GPS TX
```

##### Analog (Line Following Sensors - BBB only)

```text
P9.39 → Sensor 1 (AIN0)
P9.40 → Sensor 2 (AIN1)
P9.37 → Sensor 3 (AIN2)
P9.38 → Sensor 4 (AIN3)
```

### 2. Power Requirements

- Main board: 5V 2A power supply
- Motors: Separate 12V supply recommended
- Sensors: Powered from 3.3V board supply

## Software Architecture

### Application Components

1. **Sensor Manager**: Handles all sensor data acquisition
2. **Motor Controller**: PWM-based motor control with PID
3. **Navigation Engine**: Path planning and obstacle avoidance
4. **Vision Processor**: OpenCV-based image processing
5. **Communication Hub**: UART GPS and web interface
6. **State Machine**: High-level behavior control

### Real-time Considerations

- **PRU Integration**: Critical timing handled by Programmable Real-time Units
- **Interrupt Handling**: Fast sensor response times
- **Memory Management**: Minimal allocation in control loops
- **Priority Scheduling**: Real-time task prioritization

## Build Configuration Details

### Yocto Project Configuration Highlights

#### Target Architecture

```bitbake
MACHINE = "beaglebone-yocto"
DEFAULTTUNE = "cortexa8hf-neon"
```

#### Essential Packages

```bitbake
IMAGE_INSTALL:append = " \
    opencv \
    libgpiod \
    i2c-tools \
    spitools \
    python3 \
    robotics-controller \
"
```

#### Kernel Configuration

```bitbake
KERNEL_DEVICETREE = "am335x-boneblack.dtb"
MACHINE_FEATURES += "usbhost rtc i2c spi gpio"
DISTRO_FEATURES:append = " opencv wifi bluetooth"
```

## Testing and Validation

### Unit Tests

```bash
# Run component tests
cd tests/
./run-unit-tests.sh
```

### Hardware Tests

```bash
# Test individual interfaces
./test-gpio.sh
./test-i2c.sh
./test-spi.sh
./test-pwm.sh
```

### Integration Tests

```bash
# Full system test
./test-robot-behavior.sh
```

## Performance Metrics

### System Specifications

- **Boot Time**: < 10 seconds
- **Image Size**: < 100MB
- **RAM Usage**: < 96MB at runtime
- **Control Loop**: 100Hz update rate
- **Sensor Latency**: < 5ms

### Real-time Performance

- **Motor Response**: < 1ms
- **Obstacle Detection**: < 10ms
- **Vision Processing**: 30 FPS
- **GPS Update**: 1Hz

## Web Interface

Access the robot control interface at `http://robot-ip:8080`

### Features

- Live sensor data display
- Manual motor control
- Camera stream viewer
- Configuration settings
- System status monitoring

## Troubleshooting

### Common Issues

#### Build Problems

```bash
# Clean and rebuild
./scripts/clean.sh
source ./scripts/build.sh
bitbake robotics-controller-image -c cleansstate && bitbake robotics-controller-image
```

#### SD Card Boot Issues

- Verify image was flashed correctly
- Check SD card for corruption
- Ensure proper power supply

#### Sensor Communication

- Verify wiring connections
- Check I2C/SPI device addresses
- Test with command-line tools

### Debug Commands

```bash
# Check I2C devices
i2cdetect -y 1

# Test GPIO
gpioset gpiochip0 13=1

# Monitor system logs
dmesg | tail -f
```

## Advanced Features

### Virtual Testing with QEMU

This project fully supports QEMU-based virtual testing, allowing development and testing without physical hardware.

#### Setting Up QEMU

```bash
# Build with QEMU support
./scripts/build.sh --qemu

# Start the QEMU virtual machine
./scripts/run.sh
```

#### QEMU Features

- **Full System Emulation**: Tests the complete software stack
- **Network Support**: Connect to host network with `--network` option
- **Graphical Interface**: Enable GUI for visual testing with `--graphics`
- **Debug Mode**: Enable debug output with `--debug`
- **Performance Tuning**: Adjust memory allocation with `--memory 1G`

#### Limitations

- No hardware-specific features (PRUs, actual GPIO)
- Limited sensor simulation

### PRU Programming (BeagleBone Black)

- Real-time motor control at microsecond precision
- Hardware PWM generation
- High-speed sensor sampling

### Computer Vision Pipeline

- Object detection and tracking
- Color-based navigation
- Edge detection for line following

### Navigation Algorithms

- PID motor control
- Kalman filtering for sensor fusion
- A* pathfinding for complex navigation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Yocto Project community for the excellent embedded Linux framework
- BeagleBone.org for comprehensive hardware documentation
- OpenCV project for computer vision capabilities

## Support

For questions and support:

- Create an issue in the GitHub repository
- Check the [troubleshooting guide](docs/troubleshooting.md)
- Review the [hardware setup documentation](docs/hardware-setup.md)

---

### Happy Robot Building! 🤖
