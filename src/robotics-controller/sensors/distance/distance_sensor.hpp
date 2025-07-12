#pragma once

#include "../sensor_manager.hpp"
#include <gpiod.h>

namespace robotics {

/**
 * @brief Distance sensor using HC-SR04 ultrasonic sensor
 *
 * Measures distance using ultrasonic pulses via GPIO pins.
 */
class DistanceSensor : public Sensor {
public:
    DistanceSensor();
    ~DistanceSensor() override;

    // Rule of five: delete copy, default move
    DistanceSensor(const DistanceSensor&) = delete;
    DistanceSensor& operator=(const DistanceSensor&) = delete;
    DistanceSensor(DistanceSensor&&) noexcept = default;
    DistanceSensor& operator=(DistanceSensor&&) noexcept = default;

    bool initialize() override;
    void update() override;
    void shutdown() override;
    std::string get_name() const override;
    bool is_ready() const override;
    std::map<std::string, double> get_data() const override;

private:
    /**
     * @brief Perform actual distance measurement
     * @return Distance in cm, or -1 on error
     */
    double measure_distance();

    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
