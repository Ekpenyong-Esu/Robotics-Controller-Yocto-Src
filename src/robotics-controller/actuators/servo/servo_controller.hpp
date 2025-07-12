#pragma once

#include <memory>
#include <string>
#include <gpiod.h>

namespace robotics {

/**
 * @brief Servo motor controller using PWM
 *
 * Controls servo position using PWM signals (50Hz, 1-2ms pulse width).
 */
class ServoController {
public:
    ServoController();
    ~ServoController();

    // Rule of five: delete copy, default move
    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;
    ServoController(ServoController&&) noexcept = default;
    ServoController& operator=(ServoController&&) noexcept = default;

    bool initialize();
    void update();
    void shutdown();

    /**
     * @brief Set servo angle
     * @param angle Angle in degrees (0-180)
     */
    void set_angle(double angle);

    /**
     * @brief Get current servo angle
     * @return Current angle in degrees
     */
    double get_angle() const;

    /**
     * @brief Get servo status
     * @return Status string
     */
    std::string get_status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
