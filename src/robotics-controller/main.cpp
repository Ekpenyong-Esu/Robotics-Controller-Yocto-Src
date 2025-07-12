#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>

// Core system components
#include "core/system_manager.hpp"
#include "core/state_machine.hpp"

// Sensor components
#include "sensors/sensor_manager.hpp"

// Actuator components
#include "actuators/actuator_manager.hpp"

// Communication components
#include "communication/communication_hub.hpp"

// Navigation and vision
#include "navigation/navigation_engine.hpp"
#include "vision/vision_processor.hpp"

// Configuration
#include "config/version.hpp"
volatile bool running = true;

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "==================================================" << std::endl;
    std::cout << "Robotics Controller v" << ROBOTICS_VERSION_MAJOR
              << "." << ROBOTICS_VERSION_MINOR
              << "." << ROBOTICS_VERSION_PATCH << std::endl;
    std::cout << "==================================================" << std::endl;

    // Set up signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        // Initialize core system manager
        auto system_manager = std::make_unique<robotics::SystemManager>();

        // Initialize state machine
        auto state_machine = std::make_unique<robotics::StateMachine>();

        // Initialize sensor manager
        auto sensor_manager = std::make_unique<robotics::SensorManager>();

        // Initialize actuator manager
        auto actuator_manager = std::make_unique<robotics::ActuatorManager>();

        // Initialize communication hub
        auto communication_hub = std::make_unique<robotics::CommunicationHub>();

        // Initialize navigation engine
        auto navigation_engine = std::make_unique<robotics::NavigationEngine>();

        // Initialize vision processor
        auto vision_processor = std::make_unique<robotics::VisionProcessor>();

        std::cout << "Initializing robotics controller..." << std::endl;

        // Initialize all components
        if (!system_manager->initialize()) {
            std::cerr << "Failed to initialize system manager" << std::endl;
            return 1;
        }

        if (!sensor_manager->initialize()) {
            std::cerr << "Failed to initialize sensor manager" << std::endl;
            return 1;
        }

        if (!actuator_manager->initialize()) {
            std::cerr << "Failed to initialize actuator manager" << std::endl;
            return 1;
        }

        if (!communication_hub->initialize()) {
            std::cerr << "Failed to initialize communication hub" << std::endl;
            return 1;
        }

        if (!navigation_engine->initialize()) {
            std::cerr << "Failed to initialize navigation engine" << std::endl;
            return 1;
        }

        if (!vision_processor->initialize()) {
            std::cerr << "Failed to initialize vision processor" << std::endl;
            return 1;
        }

        std::cout << "All components initialized successfully!" << std::endl;
        std::cout << "Starting main control loop..." << std::endl;

        // Main control loop
        const auto loop_duration = std::chrono::milliseconds(100); // 10Hz

        while (running) {
            auto loop_start = std::chrono::steady_clock::now();

            // Update sensors
            sensor_manager->update();

            // Process navigation
            navigation_engine->update();

            // Process vision (if enabled)
            if (vision_processor->is_enabled()) {
                vision_processor->update();
            }

            // Update state machine
            state_machine->update();

            // Update actuators
            actuator_manager->update();

            // Handle communication
            communication_hub->update();

            // Sleep for remainder of loop time
            auto loop_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);

            if (elapsed < loop_duration) {
                std::this_thread::sleep_for(loop_duration - elapsed);
            }
        }

        std::cout << "Shutting down robotics controller..." << std::endl;

        // Shutdown all components
        vision_processor->shutdown();
        navigation_engine->shutdown();
        communication_hub->shutdown();
        actuator_manager->shutdown();
        sensor_manager->shutdown();
        system_manager->shutdown();

        std::cout << "Shutdown complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
