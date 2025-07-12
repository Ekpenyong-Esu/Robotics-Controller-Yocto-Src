#pragma once

#include "../sensor_manager.hpp"
#include <gpiod.h>

namespace robotics {

/**
 * @brief Button sensor for user input
 *
 * Handles button press detection using GPIO.
 */
class ButtonSensor : public Sensor {
public:
    ButtonSensor();
    ~ButtonSensor() override;

    // Delete copy and move constructors and assignment operators
    ButtonSensor(const ButtonSensor&) = delete;
    ButtonSensor& operator=(const ButtonSensor&) = delete;
    ButtonSensor(ButtonSensor&&) = delete;
    ButtonSensor& operator=(ButtonSensor&&) = delete;

    bool initialize() override;
    void update() override;
    void shutdown() override;
    std::string get_name() const override;
    bool is_ready() const override;
    std::map<std::string, double> get_data() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
