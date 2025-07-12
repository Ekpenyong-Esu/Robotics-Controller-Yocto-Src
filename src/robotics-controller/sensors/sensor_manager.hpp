#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>

namespace robotics {

/**
 * @brief Base class for all sensors
 */
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual bool initialize() = 0;
    virtual void update() = 0;
    virtual void shutdown() = 0;
    virtual std::string get_name() const = 0;
    virtual bool is_ready() const = 0;
    virtual std::map<std::string, double> get_data() const = 0;
};

/**
 * @brief Sensor manager for coordinating all sensors
 *
 * The SensorManager handles initialization, updates, and data collection
 * from all connected sensors in the robotics system.
 */
class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    /**
     * @brief Initialize sensor manager and all sensors
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Update all sensors (called in main loop)
     */
    void update();

    /**
     * @brief Shutdown sensor manager and all sensors
     */
    void shutdown();

    /**
     * @brief Get sensor data by name
     * @param sensor_name Name of the sensor
     * @return Sensor data map or empty map if not found
     */
    std::map<std::string, double> get_sensor_data(const std::string& sensor_name) const;

    /**
     * @brief Get all sensor data
     * @return Map of all sensor data
     */
    std::map<std::string, std::map<std::string, double>> get_all_sensor_data() const;

    /**
     * @brief Get list of available sensors
     * @return Vector of sensor names
     */
    std::vector<std::string> get_sensor_names() const;

    /**
     * @brief Check if sensor is ready
     * @param sensor_name Name of the sensor
     * @return true if sensor exists and is ready
     */
    bool is_sensor_ready(const std::string& sensor_name) const;

    /**
     * @brief Get sensor manager status
     * @return Status string
     */
    std::string get_status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
