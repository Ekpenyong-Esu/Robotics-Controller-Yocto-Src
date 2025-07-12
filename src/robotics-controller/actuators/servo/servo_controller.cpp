#include "servo_controller.hpp"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <cmath>

namespace robotics {

struct ServoController::Impl {
    gpiod_chip* chip = nullptr;
    gpiod_line* servo_line = nullptr;

    // Servo configuration
    int servo_pin = 17;  // GPIO pin for servo PWM
    double current_angle = 90.0;  // Center position
    double target_angle = 90.0;
    bool ready = false;
    std::string status = "stopped";

    // PWM parameters for servo control
    static constexpr int PWM_FREQUENCY = 50;  // 50Hz for servo
    static constexpr double MIN_PULSE_WIDTH = 1.0;  // 1ms for 0 degrees
    static constexpr double MAX_PULSE_WIDTH = 2.0;  // 2ms for 180 degrees
    static constexpr double PULSE_PERIOD = 20.0;    // 20ms period (50Hz)

    ~Impl() {
        if (servo_line) {
            gpiod_line_release(servo_line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }

    void setup_servo_pwm() {
#ifdef HAVE_GPIOD
        // Setup PWM via sysfs for servo control
        std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(servo_pin);
        std::string export_path = "/sys/class/pwm/pwmchip0/export";

        // Export PWM
        std::ofstream export_file(export_path);
        if (export_file.is_open()) {
            export_file << servo_pin;
            export_file.close();
        }

        // Set period for 50Hz (20ms = 20,000,000 nanoseconds)
        std::string period_path = pwm_path + "/period";
        std::ofstream period_file(period_path);
        if (period_file.is_open()) {
            period_file << "20000000";  // 20ms in nanoseconds
            period_file.close();
        }

        // Enable PWM
        std::string enable_path = pwm_path + "/enable";
        std::ofstream enable_file(enable_path);
        if (enable_file.is_open()) {
            enable_file << "1";
            enable_file.close();
        }

        // Set initial position (center)
        set_pwm_angle(90.0);
#endif
    }

    void set_pwm_angle(double angle) {
        // Clamp angle to valid range
        angle = std::clamp(angle, 0.0, 180.0);

#ifdef HAVE_GPIOD
        // Calculate pulse width for the angle
        // 0 degrees = 1ms, 180 degrees = 2ms
        double pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);

        // Convert to nanoseconds
        int duty_cycle_ns = static_cast<int>(pulse_width * 1000000);

        std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(servo_pin);
        std::string duty_path = pwm_path + "/duty_cycle";

        std::ofstream duty_file(duty_path);
        if (duty_file.is_open()) {
            duty_file << duty_cycle_ns;
            duty_file.close();
        }
#endif

        current_angle = angle;
    }
};

ServoController::ServoController() : pimpl_(std::make_unique<Impl>()) {}

ServoController::~ServoController() = default;

bool ServoController::initialize() {
    std::cout << "Initializing Servo Controller..." << std::endl;

#ifdef HAVE_GPIOD
    // Open GPIO chip
    pimpl_->chip = gpiod_chip_open_by_name("gpiochip0");
    if (!pimpl_->chip) {
        std::cerr << "Failed to open GPIO chip for servo controller" << std::endl;
        return false;
    }

    // Get servo line (though we use PWM via sysfs)
    pimpl_->servo_line = gpiod_chip_get_line(pimpl_->chip, pimpl_->servo_pin);
    if (!pimpl_->servo_line) {
        std::cerr << "Failed to get servo GPIO line " << pimpl_->servo_pin << std::endl;
        return false;
    }

    // Setup PWM for servo
    pimpl_->setup_servo_pwm();

    std::cout << "Servo Controller initialized on GPIO " << pimpl_->servo_pin << std::endl;
#else
    std::cout << "Servo Controller initialized (simulation mode - no GPIO)" << std::endl;
#endif

    pimpl_->ready = true;
    pimpl_->status = "ready";
    return true;
}

void ServoController::update() {
    if (!pimpl_->ready) {
        return;
    }

    // Smooth movement towards target angle
    if (std::abs(pimpl_->current_angle - pimpl_->target_angle) > 1.0) {
        double step = (pimpl_->target_angle - pimpl_->current_angle) * 0.1;  // 10% step
        double new_angle = pimpl_->current_angle + step;

        pimpl_->set_pwm_angle(new_angle);

        pimpl_->status = "moving to " + std::to_string(static_cast<int>(pimpl_->target_angle)) + "°";
    } else {
        pimpl_->status = "at " + std::to_string(static_cast<int>(pimpl_->current_angle)) + "°";
    }
}

void ServoController::shutdown() {
    std::cout << "Shutting down Servo Controller..." << std::endl;

    // Move to center position
    set_angle(90.0);

    pimpl_->ready = false;
    pimpl_->status = "stopped";
}

void ServoController::set_angle(double angle) {
    if (!pimpl_->ready) {
        return;
    }

    // Clamp to valid range
    angle = std::clamp(angle, 0.0, 180.0);
    pimpl_->target_angle = angle;

    // For immediate movement, set directly
    pimpl_->set_pwm_angle(angle);
}

double ServoController::get_angle() const {
    return pimpl_->current_angle;
}

std::string ServoController::get_status() const {
    return pimpl_->status;
}

} // namespace robotics
