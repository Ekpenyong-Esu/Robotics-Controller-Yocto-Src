#include "button_sensor.hpp"
#include <iostream>
#include <chrono>

namespace robotics {

struct ButtonSensor::Impl {
    gpiod_chip* chip = nullptr;
    gpiod_line* button_line = nullptr;
    bool ready = false;
    bool pressed = false;
    bool last_pressed = false;
    std::chrono::steady_clock::time_point last_press_time;
    int gpio_pin = 19; // Default from config

    ~Impl() {
        if (button_line) {
            gpiod_line_release(button_line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }
};

ButtonSensor::ButtonSensor() : pimpl_(std::make_unique<Impl>()) {}

ButtonSensor::~ButtonSensor() = default;

bool ButtonSensor::initialize() {
    std::cout << "Initializing Button Sensor..." << std::endl;

    // Open GPIO chip
    pimpl_->chip = gpiod_chip_open_by_name("gpiochip0");
    if (!pimpl_->chip) {
        std::cerr << "Failed to open GPIO chip for button sensor" << std::endl;
        return false;
    }

    // Get button line
    pimpl_->button_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->gpio_pin);
    if (!pimpl_->button_line) {
        std::cerr << "Failed to get GPIO line " << pimpl_->gpio_pin << " for button sensor" << std::endl;
        return false;
    }

    // Configure as input with pull-up
    if (gpiod_line_request_input(pimpl_->button_line, "button_sensor") < 0) {
        std::cerr << "Failed to configure button GPIO as input" << std::endl;
        return false;
    }

    pimpl_->ready = true;
    std::cout << "Button Sensor initialized on GPIO " << pimpl_->gpio_pin << std::endl;
    return true;
}

void ButtonSensor::update() {
    if (!pimpl_->ready) return;

    // Read button state (assuming active low with pull-up)
    int value = gpiod_line_get_value(pimpl_->button_line);
    pimpl_->last_pressed = pimpl_->pressed;
    pimpl_->pressed = (value == 0); // Active low

    // Track press time for debouncing
    if (pimpl_->pressed && !pimpl_->last_pressed) {
        pimpl_->last_press_time = std::chrono::steady_clock::now();
    }
}

void ButtonSensor::shutdown() {
    std::cout << "Shutting down Button Sensor..." << std::endl;
    pimpl_->ready = false;
}

std::string ButtonSensor::get_name() const {
    return "button";
}

bool ButtonSensor::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> ButtonSensor::get_data() const {
    std::map<std::string, double> data;
    data["pressed"] = pimpl_->pressed ? 1.0 : 0.0;
    data["pressed_changed"] = (pimpl_->pressed != pimpl_->last_pressed) ? 1.0 : 0.0;

    // Calculate press duration if currently pressed
    if (pimpl_->pressed) {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - pimpl_->last_press_time).count();
        data["press_duration_ms"] = static_cast<double>(duration);
    } else {
        data["press_duration_ms"] = 0.0;
    }

    return data;
}

} // namespace robotics
