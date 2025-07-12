#include "line_sensor_array.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace robotics {

struct LineSensorArray::Impl {
    bool ready = false;
    std::vector<double> sensor_values;
    std::vector<double> calibration_min;
    std::vector<double> calibration_max;

    // Configuration
    static constexpr int NUM_SENSORS = 8;
    std::vector<std::string> adc_paths;

    // Line detection
    double line_position = 0.0;  // -1.0 (left) to 1.0 (right)
    bool line_detected = false;
    double line_confidence = 0.0;

    Impl() {
        sensor_values.resize(NUM_SENSORS, 0.0);
        calibration_min.resize(NUM_SENSORS, 0.0);
        calibration_max.resize(NUM_SENSORS, 4095.0);  // 12-bit ADC

        // Initialize ADC paths for BeagleBone Black
        adc_paths = {
            "/sys/bus/iio/devices/iio:device0/in_voltage0_raw",  // AIN0
            "/sys/bus/iio/devices/iio:device0/in_voltage1_raw",  // AIN1
            "/sys/bus/iio/devices/iio:device0/in_voltage2_raw",  // AIN2
            "/sys/bus/iio/devices/iio:device0/in_voltage3_raw",  // AIN3
            "/sys/bus/iio/devices/iio:device0/in_voltage4_raw",  // AIN4
            "/sys/bus/iio/devices/iio:device0/in_voltage5_raw",  // AIN5
            "/sys/bus/iio/devices/iio:device0/in_voltage6_raw",  // AIN6
            "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"   // AIN7
        };
    }

    double read_adc_channel(int channel) {
        if (channel < 0 || channel >= NUM_SENSORS) {
            return 0.0;
        }

        std::ifstream adc_file(adc_paths[channel]);
        if (!adc_file.is_open()) {
            // Simulate ADC reading if file not available
            static int sim_counter = 0;
            sim_counter++;

            // Simulate a line in the center moving slightly
            double center = NUM_SENSORS / 2.0;
            double line_center = center + std::sin(sim_counter * 0.1) * 2.0;
            double distance = std::abs(channel - line_center);

            return (distance < 1.5) ? 3000.0 : 500.0;  // High value for line detection
        }

        int raw_value;
        adc_file >> raw_value;
        adc_file.close();

        return static_cast<double>(raw_value);
    }

    void calculate_line_position() {
        // Calculate weighted average for line position
        double weighted_sum = 0.0;
        double sum = 0.0;
        bool any_sensor_active = false;

        for (int i = 0; i < NUM_SENSORS; i++) {
            // Normalize sensor value (0 = white, 1 = black line)
            double normalized = (sensor_values[i] - calibration_min[i]) /
                              (calibration_max[i] - calibration_min[i]);
            normalized = std::clamp(normalized, 0.0, 1.0);

            // Use higher values as line detection (dark line on light surface)
            if (normalized > 0.3) {  // Threshold for line detection
                any_sensor_active = true;
                weighted_sum += normalized * i;
                sum += normalized;
            }
        }

        line_detected = any_sensor_active;

        if (sum > 0) {
            // Calculate position from -1.0 (leftmost) to 1.0 (rightmost)
            double center_position = weighted_sum / sum;
            double center = (NUM_SENSORS - 1) / 2.0;
            line_position = (center_position - center) / center;
            line_confidence = sum / NUM_SENSORS;  // Confidence based on total activation
        } else {
            line_position = 0.0;
            line_confidence = 0.0;
        }
    }
};

LineSensorArray::LineSensorArray() : pimpl_(std::make_unique<Impl>()) {}

LineSensorArray::~LineSensorArray() = default;

bool LineSensorArray::initialize() {
    std::cout << "Initializing Line Sensor Array..." << std::endl;

    // Test ADC channels
    bool adc_available = false;
    for (int i = 0; i < pimpl_->NUM_SENSORS; i++) {
        std::ifstream test_file(pimpl_->adc_paths[i]);
        if (test_file.is_open()) {
            adc_available = true;
            test_file.close();
            break;
        }
    }

    if (adc_available) {
        std::cout << "Line Sensor Array initialized with ADC support" << std::endl;
    } else {
        std::cout << "Line Sensor Array initialized (simulation mode - no ADC)" << std::endl;
    }

    pimpl_->ready = true;
    return true;
}

void LineSensorArray::update() {
    if (!pimpl_->ready) {
        return;
    }

    // Read all sensor values
    for (int i = 0; i < pimpl_->NUM_SENSORS; i++) {
        pimpl_->sensor_values[i] = pimpl_->read_adc_channel(i);
    }

    // Calculate line position
    pimpl_->calculate_line_position();
}

void LineSensorArray::shutdown() {
    std::cout << "Shutting down Line Sensor Array..." << std::endl;
    pimpl_->ready = false;
}

std::string LineSensorArray::get_name() const {
    return "line_array";
}

bool LineSensorArray::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> LineSensorArray::get_data() const {
    std::map<std::string, double> data;

    // Individual sensor values
    for (int i = 0; i < pimpl_->NUM_SENSORS; i++) {
        data["sensor_" + std::to_string(i)] = pimpl_->sensor_values[i];
    }

    // Processed line information
    data["line_detected"] = pimpl_->line_detected ? 1.0 : 0.0;
    data["line_position"] = pimpl_->line_position;
    data["line_confidence"] = pimpl_->line_confidence;

    return data;
}

} // namespace robotics
