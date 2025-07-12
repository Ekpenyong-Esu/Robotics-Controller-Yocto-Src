#include "gps_sensor.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <regex>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace robotics {

struct GPSSensor::Impl {
    int uart_fd = -1;
    bool ready = false;

    // GPS data
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double speed_knots = 0.0;
    double course = 0.0;
    int satellites = 0;
    bool fix_valid = false;
    std::string uart_device = "/dev/ttyS0";  // Default UART device

    // NMEA parsing buffer
    std::string nmea_buffer;

    ~Impl() {
        if (uart_fd >= 0) {
            close(uart_fd);
        }
    }

    bool setup_uart() {
        uart_fd = open(uart_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd < 0) {
            std::cerr << "Failed to open UART device: " << uart_device << std::endl;
            return false;
        }

        // Configure UART for GPS (9600 baud, 8N1)
        struct termios options;
        tcgetattr(uart_fd, &options);

        // Set baud rate
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);

        // Configure 8N1
        options.c_cflag &= ~PARENB;   // No parity
        options.c_cflag &= ~CSTOPB;   // 1 stop bit
        options.c_cflag &= ~CSIZE;    // Clear size mask
        options.c_cflag |= CS8;       // 8 data bits
        options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        // Apply settings
        tcsetattr(uart_fd, TCSANOW, &options);

        return true;
    }

    std::string read_uart_line() {
        char buffer[256];
        int bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            nmea_buffer += buffer;

            // Look for complete NMEA sentence
            size_t end_pos = nmea_buffer.find('\n');
            if (end_pos != std::string::npos) {
                std::string line = nmea_buffer.substr(0, end_pos);
                nmea_buffer.erase(0, end_pos + 1);
                return line;
            }
        }

        return "";
    }

    bool parse_gprmc(const std::string& sentence) {
        // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        std::regex gprmc_regex(R"(\$GPRMC,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)\*([^,]*))");
        std::smatch match;

        if (std::regex_match(sentence, match, gprmc_regex)) {
            std::string status = match[2];
            if (status == "A") { // Active (valid fix)
                fix_valid = true;

                // Parse latitude
                std::string lat_str = match[3];
                std::string lat_dir = match[4];
                if (!lat_str.empty()) {
                    double lat_deg = std::stod(lat_str.substr(0, 2));
                    double lat_min = std::stod(lat_str.substr(2));
                    latitude = lat_deg + lat_min / 60.0;
                    if (lat_dir == "S") latitude = -latitude;
                }

                // Parse longitude
                std::string lon_str = match[5];
                std::string lon_dir = match[6];
                if (!lon_str.empty()) {
                    double lon_deg = std::stod(lon_str.substr(0, 3));
                    double lon_min = std::stod(lon_str.substr(3));
                    longitude = lon_deg + lon_min / 60.0;
                    if (lon_dir == "W") longitude = -longitude;
                }

                // Parse speed and course
                std::string speed_str = match[7];
                std::string course_str = match[8];

                if (!speed_str.empty()) {
                    speed_knots = std::stod(speed_str);
                }

                if (!course_str.empty()) {
                    course = std::stod(course_str);
                }

                return true;
            } else {
                fix_valid = false;
            }
        }

        return false;
    }

    bool parse_gpgga(const std::string& sentence) {
        // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        std::regex gpgga_regex(R"(\$GPGGA,([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*),([^,]*)\*([^,]*))");
        std::smatch match;

        if (std::regex_match(sentence, match, gpgga_regex)) {
            std::string fix_quality = match[6];
            std::string num_sats = match[7];
            std::string altitude_str = match[9];

            if (!fix_quality.empty()) {
                int quality = std::stoi(fix_quality);
                fix_valid = (quality > 0);
            }

            if (!num_sats.empty()) {
                satellites = std::stoi(num_sats);
            }

            if (!altitude_str.empty()) {
                altitude = std::stod(altitude_str);
            }

            return true;
        }

        return false;
    }
};

GPSSensor::GPSSensor() : pimpl_(std::make_unique<Impl>()) {}
GPSSensor::~GPSSensor() = default;

bool GPSSensor::initialize() {
    std::cout << "Initializing GPS Sensor..." << std::endl;

    // Try to open and configure UART
    if (pimpl_->setup_uart()) {
        std::cout << "GPS Sensor initialized on " << pimpl_->uart_device << std::endl;
        pimpl_->ready = true;
    } else {
        std::cout << "GPS Sensor initialization failed, running in simulation mode" << std::endl;
        // Simulate GPS data for testing
        pimpl_->latitude = 37.7749;   // San Francisco
        pimpl_->longitude = -122.4194;
        pimpl_->altitude = 50.0;
        pimpl_->fix_valid = true;
        pimpl_->satellites = 8;
        pimpl_->ready = true;
    }

    return true;
}

void GPSSensor::update() {
    if (!pimpl_->ready) {
        return;
    }

    if (pimpl_->uart_fd >= 0) {
        // Read and parse NMEA sentences
        std::string line = pimpl_->read_uart_line();
        if (!line.empty()) {
            if (line.find("$GPRMC") == 0) {
                pimpl_->parse_gprmc(line);
            } else if (line.find("$GPGGA") == 0) {
                pimpl_->parse_gpgga(line);
            }
        }
    } else {
        // Simulation mode - slightly vary position
        static int counter = 0;
        counter++;

        // Simulate movement
        pimpl_->latitude += std::sin(counter * 0.01) * 0.0001;
        pimpl_->longitude += std::cos(counter * 0.01) * 0.0001;
        pimpl_->speed_knots = std::abs(std::sin(counter * 0.1)) * 5.0;
        pimpl_->course = counter % 360;
    }
}

void GPSSensor::shutdown() {
    std::cout << "Shutting down GPS Sensor..." << std::endl;
    pimpl_->ready = false;
}

std::string GPSSensor::get_name() const {
    return "gps";
}

bool GPSSensor::is_ready() const {
    return pimpl_->ready;
}

std::map<std::string, double> GPSSensor::get_data() const {
    std::map<std::string, double> data;
    data["latitude"] = pimpl_->latitude;
    data["longitude"] = pimpl_->longitude;
    data["altitude"] = pimpl_->altitude;
    data["speed_knots"] = pimpl_->speed_knots;
    data["speed_kmh"] = pimpl_->speed_knots * 1.852; // Convert knots to km/h
    data["course"] = pimpl_->course;
    data["satellites"] = static_cast<double>(pimpl_->satellites);
    data["fix_valid"] = pimpl_->fix_valid ? 1.0 : 0.0;
    return data;
}

} // namespace robotics
