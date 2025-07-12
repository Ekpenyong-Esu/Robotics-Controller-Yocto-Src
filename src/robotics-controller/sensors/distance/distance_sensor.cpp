#include "distance_sensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace robotics {

struct DistanceSensor::Impl {
    gpiod_chip* chip = nullptr;
    gpiod_line* trigger_line = nullptr;
    gpiod_line* echo_line = nullptr;
    bool ready = false;
    double distance_cm = 0.0;
    double distance_filtered = 0.0;

    // Configuration pins
    int trigger_pin = 23;  // GPIO pin for trigger
    int echo_pin = 24;     // GPIO pin for echo

    // Filtering
    static constexpr double ALPHA = 0.7;  // Low-pass filter coefficient
    static constexpr double MAX_DISTANCE = 300.0;  // Max measurable distance in cm
    static constexpr double MIN_DISTANCE = 2.0;    // Min measurable distance in cm

    // Timing constants
    static constexpr auto TRIGGER_PULSE_US = std::chrono::microseconds(10);
    static constexpr auto MEASUREMENT_TIMEOUT_US = std::chrono::microseconds(30000);

    ~Impl() {
        if (trigger_line) {
            gpiod_line_release(trigger_line);
        }
        if (echo_line) {
            gpiod_line_release(echo_line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }
};

DistanceSensor::DistanceSensor() : pimpl_(std::make_unique<Impl>()) {}

DistanceSensor::~DistanceSensor() = default;

bool DistanceSensor::initialize() {
    std::cout << "Initializing HC-SR04 Distance Sensor..." << std::endl;

#ifdef HAVE_GPIOD
    // Open GPIO chip
    pimpl_->chip = gpiod_chip_open_by_name("gpiochip0");
    if (!pimpl_->chip) {
        std::cerr << "Failed to open GPIO chip for distance sensor" << std::endl;
        return false;
    }

    // Get trigger line
    pimpl_->trigger_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->trigger_pin);
    if (!pimpl_->trigger_line) {
        std::cerr << "Failed to get trigger GPIO line " << pimpl_->trigger_pin << std::endl;
        return false;
    }

    // Get echo line
    pimpl_->echo_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->echo_pin);
    if (!pimpl_->echo_line) {
        std::cerr << "Failed to get echo GPIO line " << pimpl_->echo_pin << std::endl;
        return false;
    }

    // Configure trigger as output
    if (gpiod_line_request_output(pimpl_->trigger_line, "distance_trigger", 0) < 0) {
        std::cerr << "Failed to configure trigger GPIO as output" << std::endl;
        return false;
    }

    // Configure echo as input
    if (gpiod_line_request_input(pimpl_->echo_line, "distance_echo") < 0) {
        std::cerr << "Failed to configure echo GPIO as input" << std::endl;
        return false;
    }

    std::cout << "HC-SR04 Distance Sensor initialized (trigger: GPIO"
              << pimpl_->trigger_pin << ", echo: GPIO" << pimpl_->echo_pin << ")" << std::endl;
#else
    std::cout << "Distance Sensor initialized (simulation mode - no GPIO)" << std::endl;
    // Simulate some distance for testing
    pimpl_->distance_cm = 50.0;
    pimpl_->distance_filtered = 50.0;
#endif

    pimpl_->ready = true;
    return true;
}

void DistanceSensor::update() {
    if (!pimpl_->ready) {
        return;
    }

#ifdef HAVE_GPIOD
    // Perform HC-SR04 measurement
    double measured_distance = measure_distance();

    if (measured_distance > 0) {
        // Validate measurement range
        if (measured_distance >= pimpl_->MIN_DISTANCE && measured_distance <= pimpl_->MAX_DISTANCE) {
            pimpl_->distance_cm = measured_distance;

            // Apply low-pass filter for noise reduction
            pimpl_->distance_filtered = pimpl_->ALPHA * pimpl_->distance_filtered +
                                      (1.0 - pimpl_->ALPHA) * measured_distance;
        }
    }
#else
    // Simulation mode - vary distance slightly
    static double sim_distance = 50.0;
    static int counter = 0;
    counter++;

    // Simulate some movement
    sim_distance += std::sin(counter * 0.1) * 2.0;
    if (sim_distance < 10.0) sim_distance = 10.0;
    if (sim_distance > 200.0) sim_distance = 200.0;

    pimpl_->distance_cm = sim_distance;
    pimpl_->distance_filtered = sim_distance;
#endif
}

double DistanceSensor::measure_distance() {
#ifdef HAVE_GPIOD
    // Send 10us trigger pulse
    gpiod_line_set_value(pimpl_->trigger_line, 1);
    std::this_thread::sleep_for(pimpl_->TRIGGER_PULSE_US);
    gpiod_line_set_value(pimpl_->trigger_line, 0);

    // Wait for echo to go high
    auto start_time = std::chrono::high_resolution_clock::now();
    auto timeout = start_time + pimpl_->MEASUREMENT_TIMEOUT_US;

    while (gpiod_line_get_value(pimpl_->echo_line) == 0) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
            return -1.0; // Timeout
        }
    }

    // Measure echo pulse duration
    auto echo_start = std::chrono::high_resolution_clock::now();

    while (gpiod_line_get_value(pimpl_->echo_line) == 1) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
            return -1.0; // Timeout
        }
    }

    auto echo_end = std::chrono::high_resolution_clock::now();

    // Calculate distance: distance = (echo_duration * speed_of_sound) / 2
    // Speed of sound = 343 m/s = 34300 cm/s
    auto echo_duration = std::chrono::duration_cast<std::chrono::microseconds>(echo_end - echo_start);
    double distance_cm = (echo_duration.count() * 34300.0) / (2.0 * 1000000.0);

    return distance_cm;
#else
    return -1.0; // Not available in simulation
#endif
}

void DistanceSensor::shutdown() {
    std::cout << "Shutting down Distance Sensor..." << std::endl;
    pimpl_->ready = false;
}

std::string DistanceSensor::get_name() const {
    return "distance";
}

bool DistanceSensor::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> DistanceSensor::get_data() const {
    std::map<std::string, double> data;
    data["distance_cm"] = pimpl_->distance_cm;
    data["distance_m"] = pimpl_->distance_cm / 100.0;
    data["distance_filtered_cm"] = pimpl_->distance_filtered;
    data["distance_filtered_m"] = pimpl_->distance_filtered / 100.0;
    data["obstacle_detected"] = (pimpl_->distance_cm < 20.0) ? 1.0 : 0.0; // Obstacle within 20cm
    return data;
}

} // namespace robotics
