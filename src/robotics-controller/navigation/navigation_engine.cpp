#include "navigation_engine.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

namespace robotics {

struct NavigationEngine::Impl {
    bool ready = false;
    bool enabled = false;
    bool autonomous_mode = false;

    // Current position and orientation
    double current_x = 0.0;
    double current_y = 0.0;
    double current_heading = 0.0; // radians

    // Target position
    double target_x = 0.0;
    double target_y = 0.0;

    // Waypoint navigation
    std::vector<std::pair<double, double>> waypoints;
    size_t current_waypoint_index = 0;
    double waypoint_tolerance = 1.0; // meters

    // PID controller parameters for heading
    double heading_kp = 2.0;
    double heading_ki = 0.1;
    double heading_kd = 0.5;
    double heading_error_sum = 0.0;
    double last_heading_error = 0.0;

    // Speed control
    double max_speed = 50.0; // percentage
    double min_speed = 20.0; // percentage
    double current_speed = 0.0;

    // Obstacle avoidance
    double obstacle_distance_threshold = 30.0; // cm
    bool obstacle_detected = false;

    // Navigation state
    enum class NavigationState {
        IDLE,
        MOVING_TO_TARGET,
        AVOIDING_OBSTACLE,
        REACHED_TARGET
    } state = NavigationState::IDLE;

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double calculate_distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    double calculate_bearing(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::atan2(dy, dx);
    }
};

NavigationEngine::NavigationEngine() : pimpl_(std::make_unique<Impl>()) {}
NavigationEngine::~NavigationEngine() = default;

bool NavigationEngine::initialize() {
    std::cout << "Initializing Navigation Engine..." << std::endl;
    pimpl_->ready = true;
    pimpl_->enabled = true;
    std::cout << "Navigation Engine initialized - Ready for autonomous navigation" << std::endl;
    return true;
}

void NavigationEngine::update() {
    if (!pimpl_->ready || !pimpl_->enabled) {
        return;
    }

    // Update navigation logic based on current state
    switch (pimpl_->state) {
        case Impl::NavigationState::IDLE:
            // Check if we have waypoints to follow
            if (!pimpl_->waypoints.empty() && pimpl_->autonomous_mode) {
                pimpl_->current_waypoint_index = 0;
                pimpl_->target_x = pimpl_->waypoints[0].first;
                pimpl_->target_y = pimpl_->waypoints[0].second;
                pimpl_->state = Impl::NavigationState::MOVING_TO_TARGET;
            }
            break;

        case Impl::NavigationState::MOVING_TO_TARGET:
            update_navigation_to_target();
            break;

        case Impl::NavigationState::AVOIDING_OBSTACLE:
            update_obstacle_avoidance();
            break;

        case Impl::NavigationState::REACHED_TARGET:
            handle_target_reached();
            break;
    }
}

void NavigationEngine::update_navigation_to_target() {
    // Calculate distance to target
    double distance_to_target = pimpl_->calculate_distance(
        pimpl_->current_x, pimpl_->current_y,
        pimpl_->target_x, pimpl_->target_y
    );

    // Check if we've reached the target
    if (distance_to_target < pimpl_->waypoint_tolerance) {
        pimpl_->state = Impl::NavigationState::REACHED_TARGET;
        return;
    }

    // Check for obstacles
    if (pimpl_->obstacle_detected) {
        pimpl_->state = Impl::NavigationState::AVOIDING_OBSTACLE;
        return;
    }

    // Calculate desired heading
    double desired_heading = pimpl_->calculate_bearing(
        pimpl_->current_x, pimpl_->current_y,
        pimpl_->target_x, pimpl_->target_y
    );

    // PID control for heading
    double heading_error = pimpl_->normalize_angle(desired_heading - pimpl_->current_heading);
    pimpl_->heading_error_sum += heading_error;
    double heading_error_derivative = heading_error - pimpl_->last_heading_error;

    double steering_correction = pimpl_->heading_kp * heading_error +
                               pimpl_->heading_ki * pimpl_->heading_error_sum +
                               pimpl_->heading_kd * heading_error_derivative;

    pimpl_->last_heading_error = heading_error;

    // Calculate motor speeds for differential drive
    double base_speed = pimpl_->max_speed * std::min(1.0, distance_to_target / 5.0);
    base_speed = std::max(base_speed, pimpl_->min_speed);

    double left_speed = base_speed - steering_correction;
    double right_speed = base_speed + steering_correction;

    // Clamp speeds
    left_speed = std::clamp(left_speed, -pimpl_->max_speed, pimpl_->max_speed);
    right_speed = std::clamp(right_speed, -pimpl_->max_speed, pimpl_->max_speed);

    // Send motor commands (this would interface with motor controller)
    set_motor_speeds(left_speed, right_speed);
}

void NavigationEngine::update_obstacle_avoidance() {
    // Simple obstacle avoidance: turn right and move forward slowly
    if (pimpl_->obstacle_detected) {
        // Turn right to avoid obstacle
        set_motor_speeds(-pimpl_->min_speed, pimpl_->min_speed);
    } else {
        // Obstacle cleared, resume navigation
        pimpl_->state = Impl::NavigationState::MOVING_TO_TARGET;
    }
}

void NavigationEngine::handle_target_reached() {
    // Stop motors
    set_motor_speeds(0.0, 0.0);

    // Move to next waypoint if available
    if (pimpl_->current_waypoint_index + 1 < pimpl_->waypoints.size()) {
        pimpl_->current_waypoint_index++;
        pimpl_->target_x = pimpl_->waypoints[pimpl_->current_waypoint_index].first;
        pimpl_->target_y = pimpl_->waypoints[pimpl_->current_waypoint_index].second;
        pimpl_->state = Impl::NavigationState::MOVING_TO_TARGET;
    } else {
        // All waypoints reached
        pimpl_->state = Impl::NavigationState::IDLE;
        pimpl_->autonomous_mode = false;
    }
}

void NavigationEngine::set_motor_speeds(double left_speed, double right_speed) {
    // This would interface with the motor controller
    // For now, just store the speeds
    pimpl_->current_speed = (std::abs(left_speed) + std::abs(right_speed)) / 2.0;
}

void NavigationEngine::update_position(double x, double y, double heading) {
    pimpl_->current_x = x;
    pimpl_->current_y = y;
    pimpl_->current_heading = heading;
}

void NavigationEngine::update_obstacle_detection(bool obstacle_detected, double distance) {
    pimpl_->obstacle_detected = obstacle_detected && (distance < pimpl_->obstacle_distance_threshold);
}

void NavigationEngine::set_target(double x, double y) {
    pimpl_->target_x = x;
    pimpl_->target_y = y;
    pimpl_->state = Impl::NavigationState::MOVING_TO_TARGET;
}

void NavigationEngine::add_waypoint(double x, double y) {
    pimpl_->waypoints.emplace_back(x, y);
}

void NavigationEngine::clear_waypoints() {
    pimpl_->waypoints.clear();
    pimpl_->current_waypoint_index = 0;
}

void NavigationEngine::start_autonomous_navigation() {
    if (!pimpl_->waypoints.empty()) {
        pimpl_->autonomous_mode = true;
        pimpl_->current_waypoint_index = 0;
        pimpl_->target_x = pimpl_->waypoints[0].first;
        pimpl_->target_y = pimpl_->waypoints[0].second;
        pimpl_->state = Impl::NavigationState::MOVING_TO_TARGET;
    }
}

void NavigationEngine::stop_navigation() {
    pimpl_->autonomous_mode = false;
    pimpl_->state = Impl::NavigationState::IDLE;
    set_motor_speeds(0.0, 0.0);
}
void NavigationEngine::shutdown() { pimpl_->ready = false; }
std::string NavigationEngine::get_status() const { return "ready"; }
bool NavigationEngine::is_enabled() const { return pimpl_->enabled; }

} // namespace robotics
