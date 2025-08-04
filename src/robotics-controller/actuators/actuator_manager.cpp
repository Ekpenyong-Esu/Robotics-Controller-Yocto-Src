#include "actuator_manager.hpp"
#include "motor/motor_controller.hpp"
#include "servo/servo_controller.hpp"
#include <iostream>
#include <sstream>
#include <cmath>

namespace robotics {

// Constants for servo control
static constexpr double SERVO_CENTER_ANGLE = 90.0; // Standard servo center position

struct ActuatorManager::Impl {
    std::unique_ptr<MotorController> motor_controller;
    std::unique_ptr<ServoController> servo_controller;

    bool ready = false;
    std::string status = "uninitialized";

    // State tracking
    double left_motor_speed = 0.0;
    double right_motor_speed = 0.0;
    double servo_angle = SERVO_CENTER_ANGLE;  // Default to center position
};

ActuatorManager::ActuatorManager() : pimpl_(std::make_unique<Impl>()) {
    pimpl_->motor_controller = std::make_unique<MotorController>();
    pimpl_->servo_controller = std::make_unique<ServoController>();
}

ActuatorManager::~ActuatorManager() = default;

bool ActuatorManager::initialize() {
    std::cout << "Initializing Actuator Manager..." << std::endl;

    // Initialize motor controller - now returns MotorError
    MotorError motor_result = pimpl_->motor_controller->initialize();
    if (motor_result != MotorError::SUCCESS) {
        std::cerr << "Failed to initialize motor controller: "
                  << MotorController::error_to_string(motor_result) << std::endl;
        pimpl_->status = "motor initialization failed";
        return false;
    }

    // Initialize servo controller - now returns ServoError
    ServoError servo_result = pimpl_->servo_controller->initialize();
    if (servo_result != ServoError::SUCCESS) {
        std::cerr << "Failed to initialize servo controller: "
                  << ServoController::error_to_string(servo_result) << std::endl;
        pimpl_->status = "servo initialization failed";
        return false;
    }

    pimpl_->ready = true;
    pimpl_->status = "ready";
    std::cout << "Actuator Manager initialized successfully" << std::endl;
    return true;
}

void ActuatorManager::update() {
    if (!pimpl_->ready) {
        return;
    }

    // Update servo controller for smooth movement
    pimpl_->servo_controller->update();

    // Update status with detailed information from both controllers
    std::string motor_status = pimpl_->motor_controller->get_status();
    std::string servo_status = pimpl_->servo_controller->get_status();

    // Get current positions for more detailed status
    auto motor_speeds = get_motor_speeds();
    double current_servo = get_current_servo_angle();
    double target_servo = get_target_servo_angle();

    std::ostringstream status_stream;
    status_stream << "Motors: " << motor_status
                  << " (L:" << motor_speeds.first << "%, R:" << motor_speeds.second << "%)";
    status_stream << ", Servo: " << servo_status;

    // Add servo position details if different from target
    if (current_servo >= 0 && target_servo >= 0) {
        if (std::abs(current_servo - target_servo) > 1.0) {
            status_stream << " (" << static_cast<int>(current_servo)
                         << "° → " << static_cast<int>(target_servo) << "°)";
        } else {
            status_stream << " (" << static_cast<int>(current_servo) << "°)";
        }
    }

    pimpl_->status = status_stream.str();
}

void ActuatorManager::shutdown() {
    std::cout << "Shutting down Actuator Manager..." << std::endl;

    // Stop all motors - now returns MotorError but we'll continue shutdown regardless
    if (pimpl_->motor_controller) {
        MotorError stop_result = pimpl_->motor_controller->stop();
        if (stop_result != MotorError::SUCCESS) {
            std::cerr << "Warning: Failed to stop motors during shutdown: "
                      << MotorController::error_to_string(stop_result) << std::endl;
        }
        pimpl_->motor_controller->shutdown();
    }

    // Stop servo - attempt to center it before shutdown
    if (pimpl_->servo_controller) {
        ServoError center_result = pimpl_->servo_controller->center();
        if (center_result != ServoError::SUCCESS) {
            std::cerr << "Warning: Failed to center servo during shutdown: "
                      << ServoController::error_to_string(center_result) << std::endl;
        }
        pimpl_->servo_controller->shutdown();
    }

    pimpl_->ready = false;
    pimpl_->status = "stopped";
}

std::string ActuatorManager::get_status() const {
    return pimpl_->status;
}

void ActuatorManager::set_motor_speeds(double left_speed, double right_speed) {
    if (!pimpl_->ready || !pimpl_->motor_controller) {
        return;
    }

    pimpl_->left_motor_speed = left_speed;
    pimpl_->right_motor_speed = right_speed;

    MotorError result = pimpl_->motor_controller->set_speed(left_speed, right_speed);
    if (result != MotorError::SUCCESS) {
        std::cerr << "Warning: Failed to set motor speeds: "
                  << MotorController::error_to_string(result) << std::endl;
        // Reset stored speeds on failure
        pimpl_->left_motor_speed = 0.0;
        pimpl_->right_motor_speed = 0.0;
    }
}

void ActuatorManager::stop_motors() {
    if (pimpl_->motor_controller) {
        MotorError result = pimpl_->motor_controller->stop();
        if (result != MotorError::SUCCESS) {
            std::cerr << "Warning: Failed to stop motors: "
                      << MotorController::error_to_string(result) << std::endl;
        } else {
            // Only reset speeds if stop was successful
            pimpl_->left_motor_speed = 0.0;
            pimpl_->right_motor_speed = 0.0;
        }
    }
}

void ActuatorManager::set_servo_angle(double angle) {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return;
    }

    ServoError result = pimpl_->servo_controller->set_angle(angle);
    if (result != ServoError::SUCCESS) {
        std::cerr << "Warning: Failed to set servo angle: "
                  << ServoController::error_to_string(result) << std::endl;
        // Don't update stored angle on failure
        return;
    }

    pimpl_->servo_angle = angle;
}

void ActuatorManager::set_servo_angle_smooth(double angle, double speed) {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return;
    }

    ServoError result = pimpl_->servo_controller->set_angle_smooth(angle, speed);
    if (result != ServoError::SUCCESS) {
        std::cerr << "Warning: Failed to set servo angle (smooth): "
                  << ServoController::error_to_string(result) << std::endl;
        return;
    }

    // For smooth movement, we track the target angle
    pimpl_->servo_angle = angle;
}

void ActuatorManager::center_servo() {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return;
    }

    ServoError result = pimpl_->servo_controller->center();
    if (result != ServoError::SUCCESS) {
        std::cerr << "Warning: Failed to center servo: "
                  << ServoController::error_to_string(result) << std::endl;
        return;
    }

    pimpl_->servo_angle = SERVO_CENTER_ANGLE; // Center position
}

double ActuatorManager::get_current_servo_angle() const {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return -1.0;
    }

    return pimpl_->servo_controller->get_current_angle();
}

double ActuatorManager::get_target_servo_angle() const {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return -1.0;
    }

    return pimpl_->servo_controller->get_target_angle();
}

std::pair<double, double> ActuatorManager::get_motor_speeds() const {
    return {pimpl_->left_motor_speed, pimpl_->right_motor_speed};
}

double ActuatorManager::get_servo_angle() const {
    // For backward compatibility, return the stored target angle
    // This maintains the original behavior of this method
    return pimpl_->servo_angle;
}

} // namespace robotics
