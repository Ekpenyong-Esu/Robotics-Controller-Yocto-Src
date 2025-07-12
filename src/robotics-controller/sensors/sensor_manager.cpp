#include "sensor_manager.hpp"
#include "button/button_sensor.hpp"
#include "distance/distance_sensor.hpp"
#include "gps/gps_sensor.hpp"
#include "imu/imu_sensor.hpp"
#include "ir/ir_sensor.hpp"
#include "line/line_sensor_array.hpp"
#include <iostream>
#include <algorithm>

namespace robotics {

struct SensorManager::Impl {
    std::vector<std::unique_ptr<Sensor>> sensors;
    std::string status = "stopped";

    void add_sensor(std::unique_ptr<Sensor> sensor) {
        sensors.push_back(std::move(sensor));
    }

    Sensor* find_sensor(const std::string& name) const {
        auto it = std::find_if(sensors.begin(), sensors.end(),
            [&name](const auto& sensor) {
                return sensor->get_name() == name;
            });
        return (it != sensors.end()) ? it->get() : nullptr;
    }
};

SensorManager::SensorManager() : pimpl_(std::make_unique<Impl>()) {}

SensorManager::~SensorManager() = default;

bool SensorManager::initialize() {
    std::cout << "Initializing Sensor Manager..." << std::endl;

    // Add all sensors
    pimpl_->add_sensor(std::make_unique<ButtonSensor>());
    pimpl_->add_sensor(std::make_unique<DistanceSensor>());
    pimpl_->add_sensor(std::make_unique<GPSSensor>());
    pimpl_->add_sensor(std::make_unique<IMUSensor>());
    pimpl_->add_sensor(std::make_unique<IRSensor>());
    pimpl_->add_sensor(std::make_unique<LineSensorArray>());

    // Initialize all sensors
    bool all_initialized = true;
    for (auto& sensor : pimpl_->sensors) {
        if (!sensor->initialize()) {
            std::cerr << "Failed to initialize sensor: " << sensor->get_name() << std::endl;
            all_initialized = false;
        } else {
            std::cout << "Initialized sensor: " << sensor->get_name() << std::endl;
        }
    }

    if (all_initialized) {
        pimpl_->status = "running";
        std::cout << "Sensor Manager initialized successfully" << std::endl;
        return true;
    }

    std::cerr << "Failed to initialize some sensors" << std::endl;
    return false;
}

void SensorManager::update() {
    // Update all sensors
    for (auto& sensor : pimpl_->sensors) {
        if (sensor->is_ready()) {
            sensor->update();
        }
    }
}

void SensorManager::shutdown() {
    std::cout << "Shutting down Sensor Manager..." << std::endl;

    // Shutdown all sensors
    for (auto& sensor : pimpl_->sensors) {
        sensor->shutdown();
    }

    pimpl_->status = "stopped";
}

std::map<std::string, double> SensorManager::get_sensor_data(const std::string& sensor_name) const {
    Sensor* sensor = pimpl_->find_sensor(sensor_name);
    if (sensor && sensor->is_ready()) {
        return sensor->get_data();
    }
    return {};
}

std::map<std::string, std::map<std::string, double>> SensorManager::get_all_sensor_data() const {
    std::map<std::string, std::map<std::string, double>> all_data;

    for (const auto& sensor : pimpl_->sensors) {
        if (sensor->is_ready()) {
            all_data[sensor->get_name()] = sensor->get_data();
        }
    }

    return all_data;
}

std::vector<std::string> SensorManager::get_sensor_names() const {
    std::vector<std::string> names;
    for (const auto& sensor : pimpl_->sensors) {
        names.push_back(sensor->get_name());
    }
    return names;
}

bool SensorManager::is_sensor_ready(const std::string& sensor_name) const {
    Sensor* sensor = pimpl_->find_sensor(sensor_name);
    return sensor && sensor->is_ready();
}

std::string SensorManager::get_status() const {
    return pimpl_->status;
}

} // namespace robotics
