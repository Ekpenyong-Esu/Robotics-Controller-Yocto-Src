#pragma once

#include <memory>
#include <string>
#include <gpiod.h>

namespace robotics {

/**
 * @brief Motor controller for DC motors with PWM speed control
 *
 * Controls motor speed and direction using GPIO pins and PWM.
 * Supports dual motor configuration for differential drive robots.
 */
class MotorController {
public:
    MotorController();
    ~MotorController();

    // Rule of five: delete copy, default move
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;
    MotorController(MotorController&&) noexcept = default;
    MotorController& operator=(MotorController&&) noexcept = default;

    bool initialize();
    void update();
    void shutdown();

    /**
     * @brief Set motor speed and direction
     * @param left_speed Speed for left motor (-100 to 100)
     * @param right_speed Speed for right motor (-100 to 100)
     */
    void set_speed(double left_speed, double right_speed);

    /**
     * @brief Stop all motors
     */
    void stop();

    /**
     * @brief Get current motor status
     * @return Status string
     */
    std::string get_status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
