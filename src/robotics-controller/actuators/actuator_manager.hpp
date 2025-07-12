#pragma once
#include <memory>
#include <string>
#include <utility>

namespace robotics {

/**
 * @brief Manager for all robot actuators (motors, servos)
 *
 * Coordinates motor and servo control for robot movement.
 */
class ActuatorManager {
public:
    ActuatorManager();
    ~ActuatorManager();

    // Rule of five: delete copy and move (due to unique_ptr member management)
    ActuatorManager(const ActuatorManager&) = delete;
    ActuatorManager& operator=(const ActuatorManager&) = delete;
    ActuatorManager(ActuatorManager&&) = delete;
    ActuatorManager& operator=(ActuatorManager&&) = delete;

    bool initialize();
    void update();
    void shutdown();
    std::string get_status() const;

    /**
     * @brief Set motor speeds for differential drive
     * @param left_speed Left motor speed (-100 to 100)
     * @param right_speed Right motor speed (-100 to 100)
     */
    void set_motor_speeds(double left_speed, double right_speed);

    /**
     * @brief Stop all motors
     */
    void stop_motors();

    /**
     * @brief Set servo angle
     * @param angle Servo angle in degrees (0-180)
     */
    void set_servo_angle(double angle);

    /**
     * @brief Get current motor speeds
     * @return Pair of (left_speed, right_speed)
     */
    std::pair<double, double> get_motor_speeds() const;

    /**
     * @brief Get current servo angle
     * @return Servo angle in degrees
     */
    double get_servo_angle() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
