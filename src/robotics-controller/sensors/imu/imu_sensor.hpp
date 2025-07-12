#pragma once

#include "../sensor_manager.hpp"
#include <vector>

namespace robotics {

/**
 * @brief IMU sensor for orientation and motion tracking
 *
 * Uses MPU-9250 (or similar) IMU via I2C for accelerometer,
 * gyroscope, and magnetometer data.
 */
class IMUSensor : public Sensor {
public:
    IMUSensor();
    ~IMUSensor() override;

    // Rule of five: delete copy, default move
    IMUSensor(const IMUSensor&) = delete;
    IMUSensor& operator=(const IMUSensor&) = delete;
    IMUSensor(IMUSensor&&) noexcept = default;
    IMUSensor& operator=(IMUSensor&&) noexcept = default;

    bool initialize() override;
    void update() override;
    void shutdown() override;
    std::string get_name() const override;
    bool is_ready() const override;
    std::map<std::string, double> get_data() const override;

    /**
     * @brief Calibrate the IMU
     * @return true if calibration successful
     */
    bool calibrate();

    /**
     * @brief Get current orientation (Euler angles)
     * @return Vector containing [roll, pitch, yaw] in degrees
     */
    std::vector<double> get_orientation() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
