#include "motor_controller.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <gpiod.h>

namespace robotics {

// Constants for PWM control
static constexpr int PWM_FREQUENCY_NS = 1000000;  // 1 kHz frequency (1,000,000 ns period)
static constexpr int PWM_DUTY_CYCLE_MAX = 100;    // Maximum duty cycle percentage
static constexpr double SPEED_MIN = -100.0;       // Minimum speed (full reverse)
static constexpr double SPEED_MAX = 100.0;        // Maximum speed (full forward)
static constexpr int GPIO_INIT_DELAY_US = 50000;  // 50ms delay for GPIO initialization

/**
 * @brief Helper class to manage GPIO operations
 *
 * This class encapsulates all GPIO-related operations to keep them
 * separate from the main motor control logic.
 */
class MotorController::GPIOManager {
public:
    GPIOManager() = default;

    ~GPIOManager() {
        cleanup();
    }

    // Disable copying and moving to prevent resource management issues
    GPIOManager(const GPIOManager&) = delete;
    GPIOManager& operator=(const GPIOManager&) = delete;
    GPIOManager(GPIOManager&&) = delete;
    GPIOManager& operator=(GPIOManager&&) = delete;

    /**
     * @brief Initialize GPIO chip
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError initialize() {
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (chip_ == nullptr) {
            std::cerr << "MotorController: Failed to open GPIO chip" << std::endl;
            return MotorError::GPIO_INIT_FAILED;
        }
        return MotorError::SUCCESS;
    }

    /**
     * @brief Get GPIO line for a specific pin
     * @param pin_number GPIO pin number
     * @return GPIO line pointer or nullptr on failure
     */
    gpiod_line* get_line(int pin_number) {
        if (chip_ == nullptr) {
            return nullptr;
        }
        return gpiod_chip_get_line(chip_, pin_number);
    }

    /**
     * @brief Configure a GPIO pin as output
     * @param line GPIO line to configure
     * @param label Label for the GPIO line
     * @return MotorError::SUCCESS on success, error code on failure
     */
    static MotorError configure_output(gpiod_line* line, const std::string& label) {
        if (line == nullptr) {
            return MotorError::GPIO_INIT_FAILED;
        }

        if (gpiod_line_request_output(line, label.c_str(), 0) < 0) {
            std::cerr << "MotorController: Failed to configure " << label << " as output" << std::endl;
            return MotorError::GPIO_INIT_FAILED;
        }

        return MotorError::SUCCESS;
    }

    /**
     * @brief Set GPIO line value
     * @param line GPIO line to set
     * @param value Value to set (0 or 1)
     * @return MotorError::SUCCESS on success, error code on failure
     */
    static MotorError set_value(gpiod_line* line, int value) {
        if (line == nullptr) {
            return MotorError::GPIO_INIT_FAILED;
        }

        if (gpiod_line_set_value(line, value) < 0) {
            return MotorError::GPIO_INIT_FAILED;
        }

        return MotorError::SUCCESS;
    }

private:
    gpiod_chip* chip_ = nullptr;

    void cleanup() {
        if (chip_ != nullptr) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    }
};

/**
 * @brief Helper class to manage PWM operations
 *
 * This class handles all PWM-related operations for motor speed control.
 */
class MotorController::PWMController {
public:
    PWMController() = default;

    /**
     * @brief Setup PWM for a specific pin
     * @param pin_number PWM pin number
     * @return MotorError::SUCCESS on success, error code on failure
     */
    static MotorError setup_pwm(int pin_number) {
        const std::string pwm_base = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pin_number);

        // Export PWM if needed
        if (!is_pwm_exported(pwm_base)) {
            if (!export_pwm(pin_number)) {
                std::cerr << "MotorController: Failed to export PWM " << pin_number << std::endl;
                return MotorError::PWM_SETUP_FAILED;
            }

            // Give the system time to create the PWM files
            usleep(GPIO_INIT_DELAY_US);
        }

        // Set frequency (1 kHz)
        if (!write_to_file(pwm_base + "/period", std::to_string(PWM_FREQUENCY_NS))) {
            std::cerr << "MotorController: Failed to set PWM period for pin " << pin_number << std::endl;
            return MotorError::PWM_SETUP_FAILED;
        }

        // Set initial duty cycle to 0%
        if (!write_to_file(pwm_base + "/duty_cycle", "0")) {
            std::cerr << "MotorController: Failed to set initial PWM duty cycle for pin " << pin_number << std::endl;
            return MotorError::PWM_SETUP_FAILED;
        }

        // Enable PWM
        if (!write_to_file(pwm_base + "/enable", "1")) {
            std::cerr << "MotorController: Failed to enable PWM for pin " << pin_number << std::endl;
            return MotorError::PWM_SETUP_FAILED;
        }

        return MotorError::SUCCESS;
    }

    /**
     * @brief Set PWM duty cycle
     * @param pin_number PWM pin number
     * @param duty_percent Duty cycle percentage (0-100)
     * @return MotorError::SUCCESS on success, error code on failure
     */
    static MotorError set_duty_cycle(int pin_number, double duty_percent) {
        // Clamp duty cycle to valid range
        duty_percent = std::clamp(duty_percent, 0.0, static_cast<double>(PWM_DUTY_CYCLE_MAX));

        // Calculate duty cycle in nanoseconds
        int duty_ns = static_cast<int>(PWM_FREQUENCY_NS * duty_percent / PWM_DUTY_CYCLE_MAX);

        const std::string duty_file = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pin_number) + "/duty_cycle";

        if (!write_to_file(duty_file, std::to_string(duty_ns))) {
            std::cerr << "MotorController: Failed to set PWM duty cycle for pin " << pin_number << std::endl;
            return MotorError::PWM_SETUP_FAILED;
        }

        return MotorError::SUCCESS;
    }

    /**
     * @brief Disable PWM for a specific pin
     * @param pin_number PWM pin number
     */
    static void disable_pwm(int pin_number) {
        const std::string enable_file = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pin_number) + "/enable";
        write_to_file(enable_file, "0");
    }

private:
    /**
     * @brief Check if PWM is already exported
     * @param pwm_base_path Base path for PWM
     * @return true if exported, false otherwise
     */
    static bool is_pwm_exported(const std::string& pwm_base_path) {
        std::ifstream check_file(pwm_base_path + "/period");
        return check_file.is_open();
    }

    /**
     * @brief Export PWM pin
     * @param pin_number PWM pin number
     * @return true on success, false on failure
     */
    static bool export_pwm(int pin_number) {
        return write_to_file("/sys/class/pwm/pwmchip0/export", std::to_string(pin_number));
    }

    /**
     * @brief Write string to file
     * @param file_path Path to file
     * @param content Content to write
     * @return true on success, false on failure
     */
    static bool write_to_file(const std::string& file_path, const std::string& content) {
        std::ofstream file(file_path);
        if (!file.is_open()) {
            return false;
        }

        file << content;
        return file.good();
    }
};

/**
 * @brief Helper class to control a single motor
 *
 * This class manages one motor's GPIO lines and provides
 * simple methods to control speed and direction.
 */
class MotorController::SingleMotor {
public:
    SingleMotor(const MotorPinConfig& config, const std::string& name)
        : config_(config)
        , name_(name)
    {}

    ~SingleMotor() {
        cleanup();
    }

    // Disable copying and moving to prevent resource management issues
    SingleMotor(const SingleMotor&) = delete;
    SingleMotor& operator=(const SingleMotor&) = delete;
    SingleMotor(SingleMotor&&) = delete;
    SingleMotor& operator=(SingleMotor&&) = delete;

    /**
     * @brief Initialize motor GPIO lines
     * @param gpio_manager GPIO manager instance
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError initialize(GPIOManager* gpio_manager) {
        if (gpio_manager == nullptr) {
            return MotorError::GPIO_INIT_FAILED;
        }

        // Get GPIO lines
        dir1_line_ = gpio_manager->get_line(config_.dir1_pin);
        dir2_line_ = gpio_manager->get_line(config_.dir2_pin);

        if (dir1_line_ == nullptr || dir2_line_ == nullptr) {
            std::cerr << "MotorController: Failed to get GPIO lines for " << name_ << " motor" << std::endl;
            return MotorError::GPIO_INIT_FAILED;
        }

        // Configure as outputs
        MotorError error = GPIOManager::configure_output(dir1_line_, name_ + "_dir1");
        if (error != MotorError::SUCCESS) {
            return error;
        }

        error = GPIOManager::configure_output(dir2_line_, name_ + "_dir2");
        if (error != MotorError::SUCCESS) {
            return error;
        }

        return MotorError::SUCCESS;
    }

    /**
     * @brief Set motor speed and direction
     * @param speed Speed (-100 to 100, negative = reverse)
     * @param gpio_manager GPIO manager instance
     * @param pwm_controller PWM controller instance
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError set_speed(double speed, GPIOManager* gpio_manager, PWMController* pwm_controller) {
        if (gpio_manager == nullptr || pwm_controller == nullptr || dir1_line_ == nullptr || dir2_line_ == nullptr) {
            return MotorError::NOT_INITIALIZED;
        }

        current_speed_ = speed;

        // Set direction based on speed sign
        if (speed > 0) {
            // Forward direction
            MotorError error = GPIOManager::set_value(dir1_line_, 1);
            if (error != MotorError::SUCCESS) {
                return error;
            }
            error = GPIOManager::set_value(dir2_line_, 0);
            if (error != MotorError::SUCCESS) {
                return error;
            }
        } else if (speed < 0) {
            // Reverse direction
            MotorError error = GPIOManager::set_value(dir1_line_, 0);
            if (error != MotorError::SUCCESS) {
                return error;
            }
            error = GPIOManager::set_value(dir2_line_, 1);
            if (error != MotorError::SUCCESS) {
                return error;
            }
        } else {
            // Stop (brake)
            MotorError error = GPIOManager::set_value(dir1_line_, 0);
            if (error != MotorError::SUCCESS) {
                return error;
            }
            error = GPIOManager::set_value(dir2_line_, 0);
            if (error != MotorError::SUCCESS) {
                return error;
            }
        }

        // Set PWM duty cycle based on absolute speed
        return PWMController::set_duty_cycle(config_.pwm_pin, std::abs(speed));
    }

    /**
     * @brief Get current motor speed
     * @return Current speed (-100 to 100)
     */
    double get_speed() const {
        return current_speed_;
    }

    /**
     * @brief Get motor name
     * @return Motor name
     */
    const std::string& get_name() const {
        return name_;
    }

private:
    MotorPinConfig config_;
    std::string name_;
    double current_speed_ = 0.0;
    gpiod_line* dir1_line_ = nullptr;
    gpiod_line* dir2_line_ = nullptr;

    void cleanup() {
        if (dir1_line_ != nullptr) {
            gpiod_line_release(dir1_line_);
            dir1_line_ = nullptr;
        }
        if (dir2_line_ != nullptr) {
            gpiod_line_release(dir2_line_);
            dir2_line_ = nullptr;
        }
    }
};

// MotorController implementation

MotorController::MotorController()
    : is_initialized_(false)
    , gpio_manager_(std::make_unique<GPIOManager>())
    , pwm_controller_(std::make_unique<PWMController>())
    , left_motor_(std::make_unique<SingleMotor>(config_.left_motor, "left"))
    , right_motor_(std::make_unique<SingleMotor>(config_.right_motor, "right"))
{
    std::cout << "MotorController: Created with default configuration" << std::endl;
}

MotorController::MotorController(const MotorControllerConfig& config)
    : config_(config)
    , is_initialized_(false)
    , gpio_manager_(std::make_unique<GPIOManager>())
    , pwm_controller_(std::make_unique<PWMController>())
    , left_motor_(std::make_unique<SingleMotor>(config_.left_motor, "left"))
    , right_motor_(std::make_unique<SingleMotor>(config_.right_motor, "right"))
{
    std::cout << "MotorController: Created with custom configuration" << std::endl;
}

MotorController::~MotorController() {
    shutdown();
    std::cout << "MotorController: Destroyed" << std::endl;
}

MotorController::MotorController(MotorController&& other) noexcept
    : config_(std::move(other.config_))
    , is_initialized_(other.is_initialized_)
    , gpio_manager_(std::move(other.gpio_manager_))
    , pwm_controller_(std::move(other.pwm_controller_))
    , left_motor_(std::move(other.left_motor_))
    , right_motor_(std::move(other.right_motor_))
{
    other.is_initialized_ = false;
}

MotorController& MotorController::operator=(MotorController&& other) noexcept {
    if (this != &other) {
        shutdown();

        config_ = std::move(other.config_);
        is_initialized_ = other.is_initialized_;
        gpio_manager_ = std::move(other.gpio_manager_);
        pwm_controller_ = std::move(other.pwm_controller_);
        left_motor_ = std::move(other.left_motor_);
        right_motor_ = std::move(other.right_motor_);

        other.is_initialized_ = false;
    }
    return *this;
}

MotorError MotorController::initialize() {
    std::cout << "MotorController: Initializing..." << std::endl;

    if (is_initialized_) {
        std::cout << "MotorController: Already initialized" << std::endl;
        return MotorError::SUCCESS;
    }

    // Initialize GPIO
    MotorError error = initialize_gpio();
    if (error != MotorError::SUCCESS) {
        return error;
    }

    // Initialize motors
    error = initialize_motors();
    if (error != MotorError::SUCCESS) {
        cleanup_resources();
        return error;
    }

    is_initialized_ = true;
    std::cout << "MotorController: Initialization successful" << std::endl;
    std::cout << "  Left motor:  PWM=" << config_.left_motor.pwm_pin
              << ", DIR1=" << config_.left_motor.dir1_pin
              << ", DIR2=" << config_.left_motor.dir2_pin << std::endl;
    std::cout << "  Right motor: PWM=" << config_.right_motor.pwm_pin
              << ", DIR1=" << config_.right_motor.dir1_pin
              << ", DIR2=" << config_.right_motor.dir2_pin << std::endl;

    return MotorError::SUCCESS;
}

bool MotorController::is_initialized() const {
    return is_initialized_;
}

MotorError MotorController::set_speed(double left_speed, double right_speed) {
    if (!is_initialized_) {
        std::cerr << "MotorController: Not initialized - call initialize() first" << std::endl;
        return MotorError::NOT_INITIALIZED;
    }

    // Clamp speeds to valid range
    left_speed = clamp_speed(left_speed);
    right_speed = clamp_speed(right_speed);

    // Set left motor speed
    MotorError error = left_motor_->set_speed(left_speed, gpio_manager_.get(), pwm_controller_.get());
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to set left motor speed" << std::endl;
        return error;
    }

    // Set right motor speed
    error = right_motor_->set_speed(right_speed, gpio_manager_.get(), pwm_controller_.get());
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to set right motor speed" << std::endl;
        return error;
    }

    return MotorError::SUCCESS;
}

MotorError MotorController::stop() {
    return set_speed(0.0, 0.0);
}

double MotorController::get_left_speed() const {
    return left_motor_ ? left_motor_->get_speed() : 0.0;
}

double MotorController::get_right_speed() const {
    return right_motor_ ? right_motor_->get_speed() : 0.0;
}

std::string MotorController::get_status() const {
    if (!is_initialized_) {
        return "not_initialized";
    }

    const double left_speed = get_left_speed();
    const double right_speed = get_right_speed();

    if (left_speed == 0.0 && right_speed == 0.0) {
        return "stopped";
    }

    std::ostringstream status;
    status << "running - Left: " << static_cast<int>(left_speed)
           << "%, Right: " << static_cast<int>(right_speed) << "%";
    return status.str();
}

std::string MotorController::error_to_string(MotorError error) {
    switch (error) {
        case MotorError::SUCCESS:
            return "Success";
        case MotorError::GPIO_INIT_FAILED:
            return "GPIO initialization failed";
        case MotorError::PWM_SETUP_FAILED:
            return "PWM setup failed";
        case MotorError::INVALID_SPEED:
            return "Invalid speed value";
        case MotorError::NOT_INITIALIZED:
            return "Motor controller not initialized";
        default:
            return "Unknown error";
    }
}

void MotorController::shutdown() {
    if (!is_initialized_) {
        return;
    }

    std::cout << "MotorController: Shutting down..." << std::endl;

    // Stop motors first
    if (left_motor_ && right_motor_) {
        stop();
    }

    cleanup_resources();
    is_initialized_ = false;

    std::cout << "MotorController: Shutdown complete" << std::endl;
}

MotorError MotorController::initialize_gpio() {
    MotorError error = gpio_manager_->initialize();
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: GPIO initialization failed" << std::endl;
        return error;
    }

    return MotorError::SUCCESS;
}

MotorError MotorController::initialize_motors() {
    // Setup PWM for both motors
    MotorError error = PWMController::setup_pwm(config_.left_motor.pwm_pin);
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to setup PWM for left motor" << std::endl;
        return error;
    }

    error = PWMController::setup_pwm(config_.right_motor.pwm_pin);
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to setup PWM for right motor" << std::endl;
        return error;
    }

    // Initialize motor GPIO lines
    error = left_motor_->initialize(gpio_manager_.get());
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to initialize left motor" << std::endl;
        return error;
    }

    error = right_motor_->initialize(gpio_manager_.get());
    if (error != MotorError::SUCCESS) {
        std::cerr << "MotorController: Failed to initialize right motor" << std::endl;
        return error;
    }

    return MotorError::SUCCESS;
}

void MotorController::cleanup_resources() {
    // Disable PWM
    if (pwm_controller_) {
        PWMController::disable_pwm(config_.left_motor.pwm_pin);
        PWMController::disable_pwm(config_.right_motor.pwm_pin);
    }

    // Motors will clean up their GPIO lines automatically in their destructors
}

double MotorController::clamp_speed(double speed) {
    return std::clamp(speed, SPEED_MIN, SPEED_MAX);
}

} // namespace robotics
