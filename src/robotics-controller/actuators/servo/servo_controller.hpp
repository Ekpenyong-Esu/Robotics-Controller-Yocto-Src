#pragma once

#include <string>
#include <memory>

// Forward declaration for GPIO library type
struct gpiod_chip;
struct gpiod_line;

namespace robotics {

/**
 * @brief Configuration for servo pin and timing parameters
 *
 * This structure holds the pin number and timing characteristics for
 * controlling a servo motor with PWM signals.
 */
struct ServoConfig {
    int pwm_pin;               // PWM pin for servo control
    double min_pulse_width_ms; // Minimum pulse width in milliseconds (typically 1.0ms)
    double max_pulse_width_ms; // Maximum pulse width in milliseconds (typically 2.0ms)
    double frequency_hz;       // PWM frequency in Hz (typically 50Hz for servos)

    // Default configuration for standard servo motors
    static constexpr int DEFAULT_PWM_PIN = 18;
    static constexpr double DEFAULT_MIN_PULSE_WIDTH = 1.0;  // 1ms for 0 degrees
    static constexpr double DEFAULT_MAX_PULSE_WIDTH = 2.0;  // 2ms for 180 degrees
    static constexpr double DEFAULT_FREQUENCY = 50.0;      // 50Hz standard servo frequency

    // Default constructor with standard servo parameters
    ServoConfig()
        : pwm_pin(DEFAULT_PWM_PIN)
        , min_pulse_width_ms(DEFAULT_MIN_PULSE_WIDTH)
        , max_pulse_width_ms(DEFAULT_MAX_PULSE_WIDTH)
        , frequency_hz(DEFAULT_FREQUENCY)
    {}

    ServoConfig(int pin, double min_pulse, double max_pulse, double freq)
        : pwm_pin(pin)
        , min_pulse_width_ms(min_pulse)
        , max_pulse_width_ms(max_pulse)
        , frequency_hz(freq)
    {}
};

/**
 * @brief Error codes for servo controller operations
 */
enum class ServoError {
    SUCCESS = 0,        // Operation successful
    GPIO_INIT_FAILED,   // Failed to initialize GPIO
    PWM_SETUP_FAILED,   // Failed to setup PWM
    INVALID_ANGLE,      // Angle value out of range
    NOT_INITIALIZED,    // Controller not initialized
    HARDWARE_ERROR      // General hardware error
};

/**
 * @brief Simple servo controller for hobby servo motors
 *
 * This class provides an easy-to-understand interface for controlling hobby servo
 * motors using PWM signals. Servo motors are commonly used for precise angular
 * positioning in robotics applications.
 *
 * Key features:
 * - Angle control from 0° to 180° (configurable range)
 * - Automatic PWM signal generation with proper timing
 * - Smooth movement with optional speed control
 * - Safe initialization and cleanup
 * - Clear error reporting
 * - Hardware abstraction for different servo types
 *
 * How servo control works:
 * - Servos use 50Hz PWM signals (20ms period)
 * - Pulse width determines position: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
 * - The servo maintains position as long as the signal is present
 *
 * Usage example:
 *   ServoController servo;
 *   if (servo.initialize() == ServoError::SUCCESS) {
 *       servo.set_angle(90.0);     // Move to center position
 *       servo.set_angle(0.0);      // Move to minimum position
 *       servo.set_angle(180.0);    // Move to maximum position
 *   }
 */
class ServoController {
public:
    /**
     * @brief Constructor with default servo configuration
     */
    ServoController();

    /**
     * @brief Constructor with custom servo configuration
     * @param config Servo pin and timing configuration
     */
    explicit ServoController(const ServoConfig& config);

    /**
     * @brief Destructor - automatically cleans up resources
     */
    ~ServoController();

    // Disable copying to prevent resource management issues
    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;

    // Allow moving
    ServoController(ServoController&&) noexcept;
    ServoController& operator=(ServoController&&) noexcept;

    /**
     * @brief Initialize the servo controller
     *
     * Sets up GPIO pins and PWM for servo control. Must be called before
     * any servo operations.
     *
     * @return ServoError::SUCCESS on success, error code on failure
     */
    ServoError initialize();

    /**
     * @brief Check if the controller is initialized and ready
     * @return true if initialized, false otherwise
     */
    bool is_initialized() const;

    /**
     * @brief Set servo angle
     *
     * Commands the servo to move to the specified angle. The angle is
     * automatically clamped to the valid range [0.0, 180.0] degrees.
     *
     * @param angle Target angle in degrees (0-180)
     * @return ServoError::SUCCESS on success, error code on failure
     */
    ServoError set_angle(double angle);

    /**
     * @brief Set servo angle with speed control
     *
     * Commands the servo to move to the specified angle with controlled speed.
     * The movement will be gradual rather than immediate.
     *
     * @param angle Target angle in degrees (0-180)
     * @param speed Movement speed factor (DEFAULT_SMOOTH_SPEED = slow, 1.0 = immediate)
     * @return ServoError::SUCCESS on success, error code on failure
     */
    ServoError set_angle_smooth(double angle, double speed = DEFAULT_SMOOTH_SPEED);

    /**
     * @brief Get current servo angle
     * @return Current angle in degrees, or -1.0 if not initialized
     */
    double get_current_angle() const;

    /**
     * @brief Get target servo angle
     * @return Target angle in degrees, or -1.0 if not initialized
     */
    double get_target_angle() const;

    /**
     * @brief Center the servo to 90 degrees
     * @return ServoError::SUCCESS on success, error code on failure
     */
    ServoError center();

    /**
     * @brief Update servo position (for smooth movement)
     *
     * Call this regularly in your main loop if using smooth movement.
     * Does nothing if not in smooth movement mode.
     */
    void update();

    /**
     * @brief Get current servo status as string
     * @return Human-readable status string
     */
    std::string get_status() const;

    /**
     * @brief Convert error code to string
     * @param error Error code to convert
     * @return Human-readable error description
     */
    static std::string error_to_string(ServoError error);

    /**
     * @brief Shutdown the controller and release resources
     *
     * Moves servo to center position and cleans up GPIO resources.
     * Called automatically by destructor, but can be called manually
     * for explicit cleanup.
     */
    void shutdown();

    // Angle constants for common positions
    static constexpr double MIN_ANGLE = 0.0;      // Minimum servo angle
    static constexpr double MAX_ANGLE = 180.0;    // Maximum servo angle
    static constexpr double CENTER_ANGLE = 90.0;  // Center servo angle
    static constexpr double DEFAULT_SMOOTH_SPEED = 0.1; // Default smooth movement speed

private:
    // Forward declarations for implementation details
    class PWMController;
    class ServoMotor;

    // Configuration and state
    ServoConfig config_;
    bool is_initialized_;

    // Servo control objects
    std::unique_ptr<PWMController> pwm_controller_;
    std::unique_ptr<ServoMotor> servo_motor_;

    // Helper methods
    ServoError initialize_pwm();
    void cleanup_resources();
    static double clamp_angle(double angle);
    static double clamp_speed(double speed);
};

} // namespace robotics
