#include "imu_sensor.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace robotics {

struct IMUSensor::Impl {
    int i2c_fd = -1;
    bool ready = false;

    // MPU-9250 I2C address
    static constexpr int MPU9250_ADDR = 0x68;

    // Raw sensor data
    int16_t accel_x_raw = 0, accel_y_raw = 0, accel_z_raw = 0;
    int16_t gyro_x_raw = 0, gyro_y_raw = 0, gyro_z_raw = 0;
    int16_t mag_x_raw = 0, mag_y_raw = 0, mag_z_raw = 0;
    int16_t temp_raw = 0;

    // Calibrated data
    double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
    double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
    double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
    double temperature = 0.0;

    // Calibration offsets
    double accel_offset_x = 0.0, accel_offset_y = 0.0, accel_offset_z = 0.0;
    double gyro_offset_x = 0.0, gyro_offset_y = 0.0, gyro_offset_z = 0.0;

    // Orientation (complementary filter)
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    std::chrono::steady_clock::time_point last_update;

    // Scale factors
    static constexpr double ACCEL_SCALE = 16384.0;  // for ±2g range
    static constexpr double GYRO_SCALE = 131.0;     // for ±250°/s range
    static constexpr double MAG_SCALE = 0.6;        // µT per LSB
    static constexpr double TEMP_SCALE = 333.87;
    static constexpr double TEMP_OFFSET = 21.0;

    // Filter constants
    static constexpr double ALPHA = 0.98;  // Complementary filter coefficient

    ~Impl() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
    }

    bool setup_i2c() {
        // Open I2C device
        i2c_fd = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd < 0) {
            std::cerr << "Failed to open I2C device" << std::endl;
            return false;
        }

        // Set I2C slave address
        if (ioctl(i2c_fd, I2C_SLAVE, MPU9250_ADDR) < 0) {
            std::cerr << "Failed to set I2C slave address" << std::endl;
            return false;
        }

        return true;
    }

    bool write_register(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        return write(i2c_fd, buffer, 2) == 2;
    }

    bool read_register(uint8_t reg, uint8_t* value) {
        if (write(i2c_fd, &reg, 1) != 1) {
            return false;
        }
        return read(i2c_fd, value, 1) == 1;
    }

    bool read_registers(uint8_t reg, uint8_t* buffer, int count) {
        if (write(i2c_fd, &reg, 1) != 1) {
            return false;
        }
        return read(i2c_fd, buffer, count) == count;
    }

    bool initialize_mpu9250() {
#ifdef HAVE_GPIOD
        // Reset device
        if (!write_register(0x6B, 0x80)) {
            std::cerr << "Failed to reset MPU-9250" << std::endl;
            return false;
        }

        usleep(100000); // Wait 100ms

        // Wake up device
        if (!write_register(0x6B, 0x00)) {
            std::cerr << "Failed to wake up MPU-9250" << std::endl;
            return false;
        }

        // Configure accelerometer (±2g)
        if (!write_register(0x1C, 0x00)) {
            std::cerr << "Failed to configure accelerometer" << std::endl;
            return false;
        }

        // Configure gyroscope (±250°/s)
        if (!write_register(0x1B, 0x00)) {
            std::cerr << "Failed to configure gyroscope" << std::endl;
            return false;
        }

        // Set sample rate (1kHz / (1 + 7) = 125Hz)
        if (!write_register(0x19, 0x07)) {
            std::cerr << "Failed to set sample rate" << std::endl;
            return false;
        }

        // Configure DLPF (Digital Low Pass Filter)
        if (!write_register(0x1A, 0x06)) {
            std::cerr << "Failed to configure DLPF" << std::endl;
            return false;
        }

        return true;
#else
        return true; // Simulation mode
#endif
    }

    void read_sensor_data() {
#ifdef HAVE_GPIOD
        uint8_t buffer[14];

        // Read accelerometer, temperature, and gyroscope data
        if (read_registers(0x3B, buffer, 14)) {
            // Parse accelerometer data
            accel_x_raw = (buffer[0] << 8) | buffer[1];
            accel_y_raw = (buffer[2] << 8) | buffer[3];
            accel_z_raw = (buffer[4] << 8) | buffer[5];

            // Parse temperature data
            temp_raw = (buffer[6] << 8) | buffer[7];

            // Parse gyroscope data
            gyro_x_raw = (buffer[8] << 8) | buffer[9];
            gyro_y_raw = (buffer[10] << 8) | buffer[11];
            gyro_z_raw = (buffer[12] << 8) | buffer[13];

            // Convert to physical units
            accel_x = (accel_x_raw / ACCEL_SCALE) - accel_offset_x;
            accel_y = (accel_y_raw / ACCEL_SCALE) - accel_offset_y;
            accel_z = (accel_z_raw / ACCEL_SCALE) - accel_offset_z;

            gyro_x = (gyro_x_raw / GYRO_SCALE) - gyro_offset_x;
            gyro_y = (gyro_y_raw / GYRO_SCALE) - gyro_offset_y;
            gyro_z = (gyro_z_raw / GYRO_SCALE) - gyro_offset_z;

            temperature = (temp_raw / TEMP_SCALE) + TEMP_OFFSET;
        }
#else
        // Simulation mode - generate realistic-ish data
        static double sim_time = 0.0;
        sim_time += 0.01; // 10ms increment

        accel_x = 0.1 * std::sin(sim_time * 0.5);
        accel_y = 0.1 * std::cos(sim_time * 0.3);
        accel_z = 9.81 + 0.05 * std::sin(sim_time);

        gyro_x = 2.0 * std::sin(sim_time * 0.2);
        gyro_y = 1.5 * std::cos(sim_time * 0.4);
        gyro_z = 0.5 * std::sin(sim_time * 0.6);

        temperature = 25.0 + 2.0 * std::sin(sim_time * 0.1);
#endif
    }

    void update_orientation() {
        auto current_time = std::chrono::steady_clock::now();
        if (last_update.time_since_epoch().count() == 0) {
            last_update = current_time;
            return;
        }

        auto dt_ms = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_update);
        double dt = dt_ms.count() / 1000000.0; // Convert to seconds
        last_update = current_time;

        // Calculate angles from accelerometer
        double accel_roll = std::atan2(accel_y, std::sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
        double accel_pitch = std::atan2(-accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;

        // Integrate gyroscope data
        roll += gyro_x * dt;
        pitch += gyro_y * dt;
        yaw += gyro_z * dt;

        // Complementary filter
        roll = ALPHA * roll + (1.0 - ALPHA) * accel_roll;
        pitch = ALPHA * pitch + (1.0 - ALPHA) * accel_pitch;

        // Keep yaw in range [0, 360)
        while (yaw >= 360.0) yaw -= 360.0;
        while (yaw < 0.0) yaw += 360.0;
    }
};

IMUSensor::IMUSensor() : pimpl_(std::make_unique<Impl>()) {}

IMUSensor::~IMUSensor() = default;

bool IMUSensor::initialize() {
    std::cout << "Initializing MPU-9250 IMU Sensor..." << std::endl;

#ifdef HAVE_GPIOD
    if (!pimpl_->setup_i2c()) {
        std::cerr << "Failed to setup I2C for IMU" << std::endl;
        return false;
    }

    if (!pimpl_->initialize_mpu9250()) {
        std::cerr << "Failed to initialize MPU-9250" << std::endl;
        return false;
    }

    std::cout << "MPU-9250 IMU initialized successfully" << std::endl;
#else
    std::cout << "IMU Sensor initialized (simulation mode - no I2C)" << std::endl;
#endif

    pimpl_->ready = true;
    pimpl_->last_update = std::chrono::steady_clock::now();
    return true;
}

void IMUSensor::update() {
    if (!pimpl_->ready) {
        return;
    }

    // Read sensor data
    pimpl_->read_sensor_data();

    // Update orientation calculation
    pimpl_->update_orientation();
}

void IMUSensor::shutdown() {
    std::cout << "Shutting down IMU Sensor..." << std::endl;
    pimpl_->ready = false;
}

std::string IMUSensor::get_name() const {
    return "imu";
}

bool IMUSensor::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> IMUSensor::get_data() const {
    std::map<std::string, double> data;

    // Acceleration data (m/s²)
    data["accel_x"] = pimpl_->accel_x * 9.81; // Convert g to m/s²
    data["accel_y"] = pimpl_->accel_y * 9.81;
    data["accel_z"] = pimpl_->accel_z * 9.81;

    // Angular velocity data (°/s)
    data["gyro_x"] = pimpl_->gyro_x;
    data["gyro_y"] = pimpl_->gyro_y;
    data["gyro_z"] = pimpl_->gyro_z;

    // Magnetic field data (µT)
    data["mag_x"] = pimpl_->mag_x;
    data["mag_y"] = pimpl_->mag_y;
    data["mag_z"] = pimpl_->mag_z;

    // Temperature (°C)
    data["temperature"] = pimpl_->temperature;

    // Orientation (degrees)
    data["roll"] = pimpl_->roll;
    data["pitch"] = pimpl_->pitch;
    data["yaw"] = pimpl_->yaw;

    return data;
}

bool IMUSensor::calibrate() {
    if (!pimpl_->ready) {
        return false;
    }

    std::cout << "Calibrating IMU... Please keep the sensor stationary." << std::endl;

    // Collect samples for calibration
    const int num_samples = 1000;
    double accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    double gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < num_samples; i++) {
        pimpl_->read_sensor_data();

        accel_sum_x += pimpl_->accel_x;
        accel_sum_y += pimpl_->accel_y;
        accel_sum_z += pimpl_->accel_z;

        gyro_sum_x += pimpl_->gyro_x;
        gyro_sum_y += pimpl_->gyro_y;
        gyro_sum_z += pimpl_->gyro_z;

        usleep(1000); // 1ms delay
    }

    // Calculate offsets
    pimpl_->accel_offset_x = accel_sum_x / num_samples;
    pimpl_->accel_offset_y = accel_sum_y / num_samples;
    pimpl_->accel_offset_z = (accel_sum_z / num_samples) - 1.0; // Subtract 1g for Z-axis

    pimpl_->gyro_offset_x = gyro_sum_x / num_samples;
    pimpl_->gyro_offset_y = gyro_sum_y / num_samples;
    pimpl_->gyro_offset_z = gyro_sum_z / num_samples;

    std::cout << "IMU calibration complete" << std::endl;
    return true;
}

std::vector<double> IMUSensor::get_orientation() const {
    return {pimpl_->roll, pimpl_->pitch, pimpl_->yaw};
}

} // namespace robotics
