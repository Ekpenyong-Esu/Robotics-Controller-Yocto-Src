#pragma once
#include <memory>
#include <string>

namespace robotics {

class NavigationEngine {
public:
    NavigationEngine();
    ~NavigationEngine();

    // Delete copy and move constructors (due to complex resource management)
    NavigationEngine(const NavigationEngine&) = delete;
    NavigationEngine& operator=(const NavigationEngine&) = delete;
    NavigationEngine(NavigationEngine&&) = delete;
    NavigationEngine& operator=(NavigationEngine&&) = delete;

    bool initialize();
    void update();
    void shutdown();
    std::string get_status() const;
    bool is_enabled() const;

    /**
     * @brief Update robot position for navigation calculations
     * @param x X coordinate (meters)
     * @param y Y coordinate (meters)
     * @param heading Current heading in radians
     */
    void update_position(double x, double y, double heading);

    /**
     * @brief Update obstacle detection status
     * @param obstacle_detected True if obstacle detected
     * @param distance Distance to obstacle in cm
     */
    void update_obstacle_detection(bool obstacle_detected, double distance);

    /**
     * @brief Set navigation target
     * @param x Target X coordinate
     * @param y Target Y coordinate
     */
    void set_target(double x, double y);

    /**
     * @brief Add waypoint to navigation path
     * @param x Waypoint X coordinate
     * @param y Waypoint Y coordinate
     */
    void add_waypoint(double x, double y);

    /**
     * @brief Clear all waypoints
     */
    void clear_waypoints();

    /**
     * @brief Start autonomous navigation through waypoints
     */
    void start_autonomous_navigation();

    /**
     * @brief Stop navigation and motors
     */
    void stop_navigation();

private:
    /**
     * @brief Update navigation towards current target
     */
    void update_navigation_to_target();

    /**
     * @brief Handle obstacle avoidance behavior
     */
    void update_obstacle_avoidance();

    /**
     * @brief Handle target reached behavior
     */
    void handle_target_reached();

    /**
     * @brief Set motor speeds for differential drive
     * @param left_speed Left motor speed (-100 to 100)
     * @param right_speed Right motor speed (-100 to 100)
     */
    void set_motor_speeds(double left_speed, double right_speed);
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
