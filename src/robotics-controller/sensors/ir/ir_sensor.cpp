#include "ir_sensor.hpp"
#include <iostream>
#include <gpiod.h>

namespace robotics {

struct IRSensor::Impl {
    gpiod_chip* chip = nullptr;
    gpiod_line* ir_line = nullptr;
    bool ready = false;
    bool object_detected = false;
    int gpio_pin = 24;  // Default GPIO pin for IR sensor

    // Detection parameters
    bool last_state = false;
    int stable_count = 0;
    static constexpr int STABILITY_THRESHOLD = 3;  // Require 3 consistent readings

    ~Impl() {
        if (ir_line) {
            gpiod_line_release(ir_line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }
};

IRSensor::IRSensor() : pimpl_(std::make_unique<Impl>()) {}
IRSensor::~IRSensor() = default;

bool IRSensor::initialize() {
    std::cout << "Initializing IR Sensor..." << std::endl;

#ifdef HAVE_GPIOD
    // Open GPIO chip
    pimpl_->chip = gpiod_chip_open_by_name("gpiochip0");
    if (!pimpl_->chip) {
        std::cerr << "Failed to open GPIO chip for IR sensor" << std::endl;
        return false;
    }

    // Get IR sensor line
    pimpl_->ir_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->gpio_pin);
    if (!pimpl_->ir_line) {
        std::cerr << "Failed to get GPIO line " << pimpl_->gpio_pin << " for IR sensor" << std::endl;
        return false;
    }

    // Configure as input
    if (gpiod_line_request_input(pimpl_->ir_line, "ir_sensor") < 0) {
        std::cerr << "Failed to configure IR sensor GPIO as input" << std::endl;
        return false;
    }

    std::cout << "IR Sensor initialized on GPIO " << pimpl_->gpio_pin << std::endl;
#else
    std::cout << "IR Sensor initialized (simulation mode - no GPIO)" << std::endl;
#endif

    pimpl_->ready = true;
    return true;
}

void IRSensor::update() {
    if (!pimpl_->ready) {
        return;
    }

#ifdef HAVE_GPIOD
    // Read IR sensor state (assuming active low)
    int value = gpiod_line_get_value(pimpl_->ir_line);
    bool current_state = (value == 0);  // Active low - object detected when low

    // Implement simple debouncing
    if (current_state == pimpl_->last_state) {
        pimpl_->stable_count++;
        if (pimpl_->stable_count >= pimpl_->STABILITY_THRESHOLD) {
            pimpl_->object_detected = current_state;
            pimpl_->stable_count = pimpl_->STABILITY_THRESHOLD;  // Cap the count
        }
    } else {
        pimpl_->stable_count = 0;
        pimpl_->last_state = current_state;
    }
#else
    // Simulation mode - alternate detection
    static int sim_counter = 0;
    sim_counter++;
    pimpl_->object_detected = (sim_counter % 100 < 20);  // Detect object 20% of the time
#endif
}

void IRSensor::shutdown() {
    std::cout << "Shutting down IR Sensor..." << std::endl;
    pimpl_->ready = false;
}

std::string IRSensor::get_name() const {
    return "ir";
}

bool IRSensor::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> IRSensor::get_data() const {
    std::map<std::string, double> data;
    data["object_detected"] = pimpl_->object_detected ? 1.0 : 0.0;
    data["raw_value"] = pimpl_->object_detected ? 0.0 : 1.0;  // Inverted for active-low
    return data;
}

} // namespace robotics
