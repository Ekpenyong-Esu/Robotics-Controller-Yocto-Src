#pragma once

#include <string>
#include <memory>

// Forward declaration for GPIO library type
struct gpiod_chip;
struct gpiod_line;

namespace robotics {

/**
 * @brief Configuration for motor pin assignments
 *
 * This structure holds the pin numbers for controlling a single motor.
 * Each motor needs one PWM pin for speed control and two direction pins.
 */
struct MotorPinConfig {
    int pwm_pin;   // PWM pin for speed control
    int dir1_pin;  // Direction control pin 1
    int dir2_pin;  // Direction control pin 2

    MotorPinConfig(int pwm, int dir1, int dir2)
        : pwm_pin(pwm), dir1_pin(dir1), dir2_pin(dir2) {}
};

/**
 * @brief Motor controller configuration
 *
 * Holds configuration for both left and right motors used in
 * differential drive robotics applications.
 */
struct MotorControllerConfig {
    MotorPinConfig left_motor;   // Left motor pin configuration
    MotorPinConfig right_motor;  // Right motor pin configuration

    // Default pin assignments for common robotics setups
    static constexpr int DEFAULT_LEFT_PWM = 18;
    static constexpr int DEFAULT_LEFT_DIR1 = 20;
    static constexpr int DEFAULT_LEFT_DIR2 = 21;
    static constexpr int DEFAULT_RIGHT_PWM = 19;
    static constexpr int DEFAULT_RIGHT_DIR1 = 22;
    static constexpr int DEFAULT_RIGHT_DIR2 = 23;

    // Default configuration for common robotics setups
    MotorControllerConfig()
        : left_motor(DEFAULT_LEFT_PWM, DEFAULT_LEFT_DIR1, DEFAULT_LEFT_DIR2)
        , right_motor(DEFAULT_RIGHT_PWM, DEFAULT_RIGHT_DIR1, DEFAULT_RIGHT_DIR2)
    {}

    MotorControllerConfig(const MotorPinConfig& left, const MotorPinConfig& right)
        : left_motor(left), right_motor(right) {}
};

/**
 * @brief Error codes for motor controller operations
 */
enum class MotorError {
    SUCCESS = 0,        // Operation successful
    GPIO_INIT_FAILED,   // Failed to initialize GPIO
    PWM_SETUP_FAILED,   // Failed to setup PWM
    INVALID_SPEED,      // Speed value out of range
    NOT_INITIALIZED     // Controller not initialized
};

/**
 * @brief Simple motor controller for DC motors with PWM speed control
 *
 * This class provides an easy-to-understand interface for controlling two DC motors
 * commonly used in differential drive robots. It handles PWM speed control and
 * direction control through GPIO pins.
 *
 * Key features:
 * - Speed control from -100% to +100% (negative = reverse)
 * - Automatic direction control based on speed sign
 * - Safe initialization and cleanup
 * - Clear error reporting
 *
 * Usage example:
 *   MotorController motors;
 *   if (motors.initialize() == MotorError::SUCCESS) {
 *       motors.set_speed(50.0, -30.0);  // Left forward 50%, right reverse 30%
 *       motors.stop();                  // Stop both motors
 *   }
 */
class MotorController {
public:
    /**
     * @brief Constructor with default pin configuration
     */
    MotorController();

    /**
     * @brief Constructor with custom pin configuration
     * @param config Motor pin configuration
     */
    explicit MotorController(const MotorControllerConfig& config);

    /**
     * @brief Destructor - automatically cleans up resources
     */
    ~MotorController();

    // Disable copying to prevent resource management issues
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    // Allow moving
    MotorController(MotorController&&) noexcept;
    MotorController& operator=(MotorController&&) noexcept;

    /**
     * @brief Initialize the motor controller
     *
     * Sets up GPIO pins and PWM for motor control. Must be called before
     * any motor operations.
     *
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError initialize();

    /**
     * @brief Check if the controller is initialized and ready
     * @return true if initialized, false otherwise
     */
    bool is_initialized() const;

    /**
     * @brief Set motor speeds and directions
     *
     * Controls both motors simultaneously. Speed values are clamped
     * to the valid range [-100.0, 100.0].
     *
     * @param left_speed Speed for left motor (-100 to 100, negative = reverse)
     * @param right_speed Speed for right motor (-100 to 100, negative = reverse)
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError set_speed(double left_speed, double right_speed);

    /**
     * @brief Stop both motors immediately
     * @return MotorError::SUCCESS on success, error code on failure
     */
    MotorError stop();

    /**
     * @brief Get current left motor speed
     * @return Current speed (-100 to 100)
     */
    double get_left_speed() const;

    /**
     * @brief Get current right motor speed
     * @return Current speed (-100 to 100)
     */
    double get_right_speed() const;

    /**
     * @brief Get current motor status as string
     * @return Human-readable status string
     */
    std::string get_status() const;

    /**
     * @brief Convert error code to string
     * @param error Error code to convert
     * @return Human-readable error description
     */
    static std::string error_to_string(MotorError error);

    /**
     * @brief Shutdown the controller and release resources
     *
     * Stops all motors and cleans up GPIO resources. Called automatically
     * by destructor, but can be called manually for explicit cleanup.
     */
    void shutdown();

private:
    // Forward declaration for implementation details
    class GPIOManager;
    class PWMController;
    class SingleMotor;

    // Configuration and state
    MotorControllerConfig config_;
    bool is_initialized_;

    // Motor control objects
    std::unique_ptr<GPIOManager> gpio_manager_;
    std::unique_ptr<PWMController> pwm_controller_;
    std::unique_ptr<SingleMotor> left_motor_;
    std::unique_ptr<SingleMotor> right_motor_;

    // Helper methods
    MotorError initialize_gpio();
    MotorError initialize_motors();
    void cleanup_resources();
    static double clamp_speed(double speed);
};

} // namespace robotics
