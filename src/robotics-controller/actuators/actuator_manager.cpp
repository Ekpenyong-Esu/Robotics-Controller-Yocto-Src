#include "actuator_manager.hpp"
#include "motor/motor_controller.hpp"
#include "servo/servo_controller.hpp"
#include <iostream>

namespace robotics {

struct ActuatorManager::Impl {
    std::unique_ptr<MotorController> motor_controller;
    std::unique_ptr<ServoController> servo_controller;
    bool ready = false;
    std::string status = "stopped";

    // Current control values
    double left_motor_speed = 0.0;
    double right_motor_speed = 0.0;
    double servo_angle = 90.0; // Center position
};

ActuatorManager::ActuatorManager() : pimpl_(std::make_unique<Impl>()) {}

ActuatorManager::~ActuatorManager() = default;

bool ActuatorManager::initialize() {
    std::cout << "Initializing Actuator Manager..." << std::endl;

    // Initialize motor controller
    pimpl_->motor_controller = std::make_unique<MotorController>();
    if (!pimpl_->motor_controller->initialize()) {
        std::cerr << "Failed to initialize motor controller" << std::endl;
        return false;
    }

    // Initialize servo controller
    pimpl_->servo_controller = std::make_unique<ServoController>();
    if (!pimpl_->servo_controller->initialize()) {
        std::cerr << "Failed to initialize servo controller" << std::endl;
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

    // Update motor controller
    pimpl_->motor_controller->update();

    // Update servo controller
    pimpl_->servo_controller->update();

    // Update status
    std::string motor_status = pimpl_->motor_controller->get_status();
    std::string servo_status = pimpl_->servo_controller->get_status();

    pimpl_->status = "Motors: " + motor_status + ", Servo: " + servo_status;
}

void ActuatorManager::shutdown() {
    std::cout << "Shutting down Actuator Manager..." << std::endl;

    // Stop all motors
    if (pimpl_->motor_controller) {
        pimpl_->motor_controller->stop();
        pimpl_->motor_controller->shutdown();
    }

    // Stop servo
    if (pimpl_->servo_controller) {
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

    pimpl_->motor_controller->set_speed(left_speed, right_speed);
}

void ActuatorManager::stop_motors() {
    if (pimpl_->motor_controller) {
        pimpl_->motor_controller->stop();
        pimpl_->left_motor_speed = 0.0;
        pimpl_->right_motor_speed = 0.0;
    }
}

void ActuatorManager::set_servo_angle(double angle) {
    if (!pimpl_->ready || !pimpl_->servo_controller) {
        return;
    }

    pimpl_->servo_angle = angle;
    pimpl_->servo_controller->set_angle(angle);
}

std::pair<double, double> ActuatorManager::get_motor_speeds() const {
    return {pimpl_->left_motor_speed, pimpl_->right_motor_speed};
}

double ActuatorManager::get_servo_angle() const {
    return pimpl_->servo_angle;
}

} // namespace robotics
