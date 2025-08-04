#include "servo_controller.hpp"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <unistd.h>
#include <gpiod.h>

namespace robotics {

// Constants for PWM control
static constexpr int PWM_PERIOD_NS = 20000000;     // 20ms period for 50Hz
static constexpr int NANOS_PER_MILLISECOND = 1000000;
static constexpr int PWM_INIT_DELAY_US = 50000;    // 50ms delay for PWM initialization
static constexpr double ANGLE_TOLERANCE = 1.0;     // Tolerance for smooth movement completion
static constexpr double MIN_SPEED = 0.01;          // Minimum movement speed
static constexpr double MAX_SPEED = 1.0;           // Maximum movement speed
static constexpr double MILLISECONDS_PER_SECOND = 1000.0; // Conversion factor
static constexpr double SERVO_MAX_ANGLE = 180.0;   // Maximum servo angle in degrees
static constexpr int SHUTDOWN_DELAY_US = 500000;   // 500ms delay for shutdown movement

/**
 * @brief Helper class to manage PWM operations for servo control
 *
 * This class handles all PWM-related operations specifically for servo motors,
 * including proper timing calculations and Linux sysfs PWM interface.
 */
class ServoController::PWMController {
public:
    PWMController(const ServoConfig& config) : config_(config) {}

    ~PWMController() {
        cleanup();
    }

    // Disable copying and moving to prevent resource management issues
    PWMController(const PWMController&) = delete;
    PWMController& operator=(const PWMController&) = delete;
    PWMController(PWMController&&) = delete;
    PWMController& operator=(PWMController&&) = delete;

    /**
     * @brief Initialize PWM for servo control
     * @return ServoError::SUCCESS on success, error code on failure
     */
    ServoError initialize() {
        const std::string pwm_base = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(config_.pwm_pin);

        // Export PWM if needed
        if (!is_pwm_exported(pwm_base)) {
            if (!export_pwm(config_.pwm_pin)) {
                std::cerr << "ServoController: Failed to export PWM " << config_.pwm_pin << std::endl;
                return ServoError::PWM_SETUP_FAILED;
            }

            // Give the system time to create the PWM files
            usleep(PWM_INIT_DELAY_US);
        }

        // Calculate period in nanoseconds
        int period_ns = static_cast<int>((MILLISECONDS_PER_SECOND / config_.frequency_hz) * NANOS_PER_MILLISECOND);

        // Set PWM period
        if (!write_to_file(pwm_base + "/period", std::to_string(period_ns))) {
            std::cerr << "ServoController: Failed to set PWM period for pin " << config_.pwm_pin << std::endl;
            return ServoError::PWM_SETUP_FAILED;
        }

        // Set initial duty cycle for center position
        if (!set_duty_cycle_for_angle(ServoController::CENTER_ANGLE)) {
            std::cerr << "ServoController: Failed to set initial PWM duty cycle" << std::endl;
            return ServoError::PWM_SETUP_FAILED;
        }

        // Enable PWM
        if (!write_to_file(pwm_base + "/enable", "1")) {
            std::cerr << "ServoController: Failed to enable PWM for pin " << config_.pwm_pin << std::endl;
            return ServoError::PWM_SETUP_FAILED;
        }

        pwm_base_path_ = pwm_base;
        is_initialized_ = true;
        return ServoError::SUCCESS;
    }

    /**
     * @brief Set PWM duty cycle for a specific angle
     * @param angle Servo angle in degrees (0-180)
     * @return true on success, false on failure
     */
    bool set_duty_cycle_for_angle(double angle) {
        if (!is_initialized_) {
            return false;
        }

        // Clamp angle to valid range
        angle = std::clamp(angle, 0.0, SERVO_MAX_ANGLE);

        // Calculate pulse width in milliseconds
        double pulse_width_ms = config_.min_pulse_width_ms +
            ((angle / SERVO_MAX_ANGLE) * (config_.max_pulse_width_ms - config_.min_pulse_width_ms));

        // Convert to nanoseconds
        int duty_cycle_ns = static_cast<int>(pulse_width_ms * NANOS_PER_MILLISECOND);

        // Write duty cycle
        return write_to_file(pwm_base_path_ + "/duty_cycle", std::to_string(duty_cycle_ns));
    }

    /**
     * @brief Disable PWM output
     */
    void disable() {
        if (is_initialized_ && !pwm_base_path_.empty()) {
            write_to_file(pwm_base_path_ + "/enable", "0");
        }
    }

private:
    ServoConfig config_;
    std::string pwm_base_path_;
    bool is_initialized_ = false;

    /**
     * @brief Check if PWM is already exported
     */
    static bool is_pwm_exported(const std::string& pwm_path) {
        std::ifstream file(pwm_path + "/period");
        return file.good();
    }

    /**
     * @brief Export PWM pin
     */
    static bool export_pwm(int pin_number) {
        return write_to_file("/sys/class/pwm/pwmchip0/export", std::to_string(pin_number));
    }

    /**
     * @brief Write string to file
     */
    static bool write_to_file(const std::string& file_path, const std::string& content) {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            return false;
        }
        file << content;
        return file.good();
    }

    void cleanup() {
        disable();
    }
};

/**
 * @brief Helper class to manage servo motor state and movement
 *
 * This class encapsulates the servo motor's state, position tracking,
 * and smooth movement logic.
 */
class ServoController::ServoMotor {
public:
    ServoMotor() = default;

    /**
     * @brief Set target angle immediately
     * @param angle Target angle in degrees
     */
    void set_angle_immediate(double angle) {
        angle = ServoController::clamp_angle(angle);
        current_angle_ = angle;
        target_angle_ = angle;
        is_moving_ = false;
        status_ = "at " + std::to_string(static_cast<int>(angle)) + "°";
    }

    /**
     * @brief Set target angle for smooth movement
     * @param angle Target angle in degrees
     * @param speed Movement speed factor (0.0-1.0)
     */
    void set_angle_smooth(double angle, double speed) {
        angle = ServoController::clamp_angle(angle);
        speed = ServoController::clamp_speed(speed);

        target_angle_ = angle;
        movement_speed_ = speed;
        is_moving_ = std::abs(target_angle_ - current_angle_) > ANGLE_TOLERANCE;

        if (is_moving_) {
            status_ = "moving to " + std::to_string(static_cast<int>(target_angle_)) + "°";
        } else {
            status_ = "at " + std::to_string(static_cast<int>(current_angle_)) + "°";
        }
    }

    /**
     * @brief Update position for smooth movement
     * @return Current angle after update
     */
    double update_position() {
        if (!is_moving_) {
            return current_angle_;
        }

        double angle_diff = target_angle_ - current_angle_;
        if (std::abs(angle_diff) <= ANGLE_TOLERANCE) {
            current_angle_ = target_angle_;
            is_moving_ = false;
            status_ = "at " + std::to_string(static_cast<int>(current_angle_)) + "°";
        } else {
            double step = angle_diff * movement_speed_;
            current_angle_ += step;
            status_ = "moving to " + std::to_string(static_cast<int>(target_angle_)) + "°";
        }

        return current_angle_;
    }

    double get_current_angle() const { return current_angle_; }
    double get_target_angle() const { return target_angle_; }
    bool is_moving() const { return is_moving_; }
    std::string get_status() const { return status_; }

private:
    double current_angle_ = ServoController::CENTER_ANGLE;
    double target_angle_ = ServoController::CENTER_ANGLE;
    double movement_speed_ = ServoController::DEFAULT_SMOOTH_SPEED;
    bool is_moving_ = false;
    std::string status_ = "initialized";
};

// ServoController implementation

ServoController::ServoController()
    : is_initialized_(false)
{}

ServoController::ServoController(const ServoConfig& config)
    : config_(config)
    , is_initialized_(false)
{}

ServoController::~ServoController() {
    shutdown();
}

ServoController::ServoController(ServoController&&) noexcept = default;
ServoController& ServoController::operator=(ServoController&&) noexcept = default;

ServoError ServoController::initialize() {
    if (is_initialized_) {
        return ServoError::SUCCESS;
    }

    std::cout << "ServoController: Initializing servo on pin " << config_.pwm_pin << std::endl;

    // Initialize PWM controller
    pwm_controller_ = std::make_unique<PWMController>(config_);
    ServoError result = pwm_controller_->initialize();
    if (result != ServoError::SUCCESS) {
        cleanup_resources();
        return result;
    }

    // Initialize servo motor state
    servo_motor_ = std::make_unique<ServoMotor>();
    servo_motor_->set_angle_immediate(CENTER_ANGLE);

    is_initialized_ = true;
    std::cout << "ServoController: Successfully initialized servo controller" << std::endl;
    return ServoError::SUCCESS;
}

bool ServoController::is_initialized() const {
    return is_initialized_;
}

ServoError ServoController::set_angle(double angle) {
    if (!is_initialized_) {
        return ServoError::NOT_INITIALIZED;
    }

    angle = clamp_angle(angle);

    if (!pwm_controller_->set_duty_cycle_for_angle(angle)) {
        return ServoError::HARDWARE_ERROR;
    }

    servo_motor_->set_angle_immediate(angle);
    return ServoError::SUCCESS;
}

ServoError ServoController::set_angle_smooth(double angle, double speed) {
    if (!is_initialized_) {
        return ServoError::NOT_INITIALIZED;
    }

    angle = clamp_angle(angle);
    speed = clamp_speed(speed);

    servo_motor_->set_angle_smooth(angle, speed);
    return ServoError::SUCCESS;
}

double ServoController::get_current_angle() const {
    if (!is_initialized_ || !servo_motor_) {
        return -1.0;
    }
    return servo_motor_->get_current_angle();
}

double ServoController::get_target_angle() const {
    if (!is_initialized_ || !servo_motor_) {
        return -1.0;
    }
    return servo_motor_->get_target_angle();
}

ServoError ServoController::center() {
    return set_angle(CENTER_ANGLE);
}

void ServoController::update() {
    if (!is_initialized_ || !servo_motor_) {
        return;
    }

    if (servo_motor_->is_moving()) {
        double current_angle = servo_motor_->update_position();
        pwm_controller_->set_duty_cycle_for_angle(current_angle);
    }
}

std::string ServoController::get_status() const {
    if (!is_initialized_) {
        return "not initialized";
    }
    if (!servo_motor_) {
        return "error: no servo motor";
    }
    return servo_motor_->get_status();
}

std::string ServoController::error_to_string(ServoError error) {
    switch (error) {
        case ServoError::SUCCESS:
            return "Success";
        case ServoError::GPIO_INIT_FAILED:
            return "GPIO initialization failed";
        case ServoError::PWM_SETUP_FAILED:
            return "PWM setup failed";
        case ServoError::INVALID_ANGLE:
            return "Invalid angle value";
        case ServoError::NOT_INITIALIZED:
            return "Controller not initialized";
        case ServoError::HARDWARE_ERROR:
            return "Hardware error";
        default:
            return "Unknown error";
    }
}

void ServoController::shutdown() {
    if (!is_initialized_) {
        return;
    }

    std::cout << "ServoController: Shutting down servo controller..." << std::endl;

    // Move to center position before shutdown
    if (pwm_controller_) {
        pwm_controller_->set_duty_cycle_for_angle(CENTER_ANGLE);
        usleep(SHUTDOWN_DELAY_US); // Wait 500ms for movement to complete
    }

    cleanup_resources();
    is_initialized_ = false;

    std::cout << "ServoController: Servo controller shutdown complete" << std::endl;
}

ServoError ServoController::initialize_pwm() {
    if (!pwm_controller_) {
        return ServoError::PWM_SETUP_FAILED;
    }
    return pwm_controller_->initialize();
}

void ServoController::cleanup_resources() {
    pwm_controller_.reset();
    servo_motor_.reset();
}

double ServoController::clamp_angle(double angle) {
    return std::clamp(angle, MIN_ANGLE, MAX_ANGLE);
}

double ServoController::clamp_speed(double speed) {
    return std::clamp(speed, MIN_SPEED, MAX_SPEED);
}

} // namespace robotics
