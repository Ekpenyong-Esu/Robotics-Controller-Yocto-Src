#include "motor_controller.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace robotics {

struct MotorController::Impl {
    gpiod_chip* chip = nullptr;

    // Left motor GPIO pins
    gpiod_line* left_dir1_line = nullptr;
    gpiod_line* left_dir2_line = nullptr;
    int left_pwm_pin = 18;  // PWM pin for left motor

    // Right motor GPIO pins
    gpiod_line* right_dir1_line = nullptr;
    gpiod_line* right_dir2_line = nullptr;
    int right_pwm_pin = 19; // PWM pin for right motor

    // Motor configuration
    int left_dir1_pin = 20;
    int left_dir2_pin = 21;
    int right_dir1_pin = 22;
    int right_dir2_pin = 23;

    // Current motor state
    double left_speed = 0.0;
    double right_speed = 0.0;
    bool running = false;
    std::string status = "stopped";

    ~Impl() {
        if (left_dir1_line) gpiod_line_release(left_dir1_line);
        if (left_dir2_line) gpiod_line_release(left_dir2_line);
        if (right_dir1_line) gpiod_line_release(right_dir1_line);
        if (right_dir2_line) gpiod_line_release(right_dir2_line);
        if (chip) gpiod_chip_close(chip);
    }

    void setup_pwm(int pin, int frequency = 1000) {
// #ifdef HAVE_GPIOD
        // Setup PWM via sysfs (this is a simplified approach)
        // In a real implementation, you might use a proper PWM library
        std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pin);
        std::string export_path = "/sys/class/pwm/pwmchip0/export";

        // Export PWM
        std::ofstream export_file(export_path);
        if (export_file.is_open()) {
            export_file << pin;
            export_file.close();
        }

        // Set period (frequency)
        std::string period_path = pwm_path + "/period";
        std::ofstream period_file(period_path);
        if (period_file.is_open()) {
            period_file << (1000000000 / frequency); // nanoseconds
            period_file.close();
        }

        // Enable PWM
        std::string enable_path = pwm_path + "/enable";
        std::ofstream enable_file(enable_path);
        if (enable_file.is_open()) {
            enable_file << "1";
            enable_file.close();
        }
// #endif
    }

    void set_pwm_duty_cycle(int pin, double duty_percent) {
// #ifdef HAVE_GPIOD
        std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pin);
        std::string duty_path = pwm_path + "/duty_cycle";

        // Calculate duty cycle in nanoseconds
        int period_ns = 1000000; // 1ms period for 1kHz
        int duty_ns = static_cast<int>(period_ns * std::abs(duty_percent) / 100.0);

        std::ofstream duty_file(duty_path);
        if (duty_file.is_open()) {
            duty_file << duty_ns;
            duty_file.close();
        }
// #endif
    }
};

MotorController::MotorController() : pimpl_(std::make_unique<Impl>()) {}

MotorController::~MotorController() = default;

bool MotorController::initialize() {
    std::cout << "Initializing Dual Motor Controller..." << std::endl;

// #ifdef HAVE_GPIOD
    // Open GPIO chip
    pimpl_->chip = gpiod_chip_open_by_name("gpiochip0");
    if (!pimpl_->chip) {
        std::cerr << "Failed to open GPIO chip for motor controller" << std::endl;
        return false;
    }

    // Initialize left motor direction pins
    pimpl_->left_dir1_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->left_dir1_pin);
    pimpl_->left_dir2_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->left_dir2_pin);

    if (!pimpl_->left_dir1_line || !pimpl_->left_dir2_line) {
        std::cerr << "Failed to get left motor direction GPIO lines" << std::endl;
        return false;
    }

    // Initialize right motor direction pins
    pimpl_->right_dir1_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->right_dir1_pin);
    pimpl_->right_dir2_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->right_dir2_pin);

    if (!pimpl_->right_dir1_line || !pimpl_->right_dir2_line) {
        std::cerr << "Failed to get right motor direction GPIO lines" << std::endl;
        return false;
    }

    // Configure direction pins as outputs
    if (gpiod_line_request_output(pimpl_->left_dir1_line, "left_motor_dir1", 0) < 0 ||
        gpiod_line_request_output(pimpl_->left_dir2_line, "left_motor_dir2", 0) < 0 ||
        gpiod_line_request_output(pimpl_->right_dir1_line, "right_motor_dir1", 0) < 0 ||
        gpiod_line_request_output(pimpl_->right_dir2_line, "right_motor_dir2", 0) < 0) {
        std::cerr << "Failed to configure motor direction pins as outputs" << std::endl;
        return false;
    }

    // Setup PWM for both motors
    pimpl_->setup_pwm(pimpl_->left_pwm_pin);
    pimpl_->setup_pwm(pimpl_->right_pwm_pin);

    std::cout << "Dual Motor Controller initialized (Left: GPIO "
              << pimpl_->left_dir1_pin << "/" << pimpl_->left_dir2_pin
              << ", PWM " << pimpl_->left_pwm_pin
              << ", Right: GPIO " << pimpl_->right_dir1_pin << "/"
              << pimpl_->right_dir2_pin << ", PWM " << pimpl_->right_pwm_pin << ")" << std::endl;
// #else
    std::cout << "Motor Controller initialized (simulation mode - no GPIO)" << std::endl;
// #endif

    pimpl_->running = true;
    pimpl_->status = "ready";
    return true;
}

void MotorController::set_speed(double left_speed, double right_speed) {
    // Clamp speeds to -100 to 100 range
    left_speed = std::clamp(left_speed, -100.0, 100.0);
    right_speed = std::clamp(right_speed, -100.0, 100.0);

    pimpl_->left_speed = left_speed;
    pimpl_->right_speed = right_speed;

// #ifdef HAVE_GPIOD
    // Set left motor direction and speed
    if (left_speed > 0) {
        // Forward
        gpiod_line_set_value(pimpl_->left_dir1_line, 1);
        gpiod_line_set_value(pimpl_->left_dir2_line, 0);
    } else if (left_speed < 0) {
        // Reverse
        gpiod_line_set_value(pimpl_->left_dir1_line, 0);
        gpiod_line_set_value(pimpl_->left_dir2_line, 1);
    } else {
        // Stop
        gpiod_line_set_value(pimpl_->left_dir1_line, 0);
        gpiod_line_set_value(pimpl_->left_dir2_line, 0);
    }

    // Set right motor direction and speed
    if (right_speed > 0) {
        // Forward
        gpiod_line_set_value(pimpl_->right_dir1_line, 1);
        gpiod_line_set_value(pimpl_->right_dir2_line, 0);
    } else if (right_speed < 0) {
        // Reverse
        gpiod_line_set_value(pimpl_->right_dir1_line, 0);
        gpiod_line_set_value(pimpl_->right_dir2_line, 1);
    } else {
        // Stop
        gpiod_line_set_value(pimpl_->right_dir1_line, 0);
        gpiod_line_set_value(pimpl_->right_dir2_line, 0);
    }

    // Set PWM duty cycles
    pimpl_->set_pwm_duty_cycle(pimpl_->left_pwm_pin, std::abs(left_speed));
    pimpl_->set_pwm_duty_cycle(pimpl_->right_pwm_pin, std::abs(right_speed));
// #endif

    if (left_speed != 0.0 || right_speed != 0.0) {
        pimpl_->status = "running";
    } else {
        pimpl_->status = "stopped";
    }
}

void MotorController::stop() {
    set_speed(0.0, 0.0);
}

void MotorController::update() {
    // Motor control update - could implement acceleration/deceleration curves here
    if (pimpl_->running) {
        // Update status based on current speeds
        if (pimpl_->left_speed != 0.0 || pimpl_->right_speed != 0.0) {
            pimpl_->status = "running - L:" + std::to_string(static_cast<int>(pimpl_->left_speed)) +
                           "% R:" + std::to_string(static_cast<int>(pimpl_->right_speed)) + "%";
        } else {
            pimpl_->status = "ready";
        }
    }
}

std::string MotorController::get_status() const {
    return pimpl_->status;
}

void MotorController::shutdown() {
    std::cout << "Shutting down Motor Controller..." << std::endl;
    pimpl_->status = "stopped";
}

} // namespace robotics
