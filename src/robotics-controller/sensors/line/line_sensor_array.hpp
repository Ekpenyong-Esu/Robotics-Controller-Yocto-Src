#pragma once
#include "../sensor_manager.hpp"
namespace robotics {
class LineSensorArray : public Sensor {
public:
    LineSensorArray();
    ~LineSensorArray() override;
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
