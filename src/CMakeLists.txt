# ==============================================================================
# Robotics Controller Project
# ==============================================================================

cmake_minimum_required(VERSION 3.14)

project(RoboticsControllerProject
    LANGUAGES CXX
    VERSION 1.0.0
    DESCRIPTION "Robotics Controller Project"
)

# CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Setup
include(RoboticsConfig)
include(RoboticsDependencies)
include(RoboticsInstallation)
robotics_setup_dependencies()
robotics_setup_installation()

# Generate version header
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/robotics-controller/config/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/robotics-controller/config/version.hpp"
    @ONLY
)

# Main component
add_subdirectory(robotics-controller)

# Web interface
add_subdirectory(web-interface)

# Install documentation
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../README.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/BUILD_SYSTEM_README.md"
    DESTINATION share/doc/robotics-controller
    OPTIONAL
)

# Install generated version header
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/robotics-controller/config/version.hpp"
    DESTINATION include/robotics-controller/config
)
