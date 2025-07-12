#include "serial_communication.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace robotics {

struct SerialCommunication::Impl {
    int serial_fd = -1;
    std::string device_path = "/dev/ttyUSB0";  // Default serial device
    int baud_rate = 9600;
    bool ready = false;
    std::string status = "stopped";

    // Communication buffers
    char rx_buffer[256];
    char tx_buffer[256];

    ~Impl() {
        if (serial_fd >= 0) {
            close(serial_fd);
        }
    }

    bool setup_serial() {
        // Open serial port
        serial_fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd < 0) {
            std::cerr << "Failed to open serial device: " << device_path << std::endl;
            return false;
        }

        // Configure serial port
        struct termios options;
        tcgetattr(serial_fd, &options);

        // Set baud rate
        switch (baud_rate) {
            case 9600:
                cfsetispeed(&options, B9600);
                cfsetospeed(&options, B9600);
                break;
            case 115200:
                cfsetispeed(&options, B115200);
                cfsetospeed(&options, B115200);
                break;
            default:
                cfsetispeed(&options, B9600);
                cfsetospeed(&options, B9600);
                break;
        }

        // Configure 8N1
        options.c_cflag &= ~PARENB;   // No parity
        options.c_cflag &= ~CSTOPB;   // 1 stop bit
        options.c_cflag &= ~CSIZE;    // Clear size mask
        options.c_cflag |= CS8;       // 8 data bits
        options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

        // Raw input/output
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        // Set timeout
        options.c_cc[VMIN] = 0;   // Non-blocking read
        options.c_cc[VTIME] = 1;  // 0.1 second timeout

        // Apply settings
        if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
            std::cerr << "Failed to configure serial port" << std::endl;
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        return true;
    }
};

SerialCommunication::SerialCommunication() : pimpl_(std::make_unique<Impl>()) {}

SerialCommunication::~SerialCommunication() = default;

bool SerialCommunication::initialize() {
    std::cout << "Initializing Serial Communication..." << std::endl;
    pimpl_->status = "running";
    return true;
}

void SerialCommunication::update() {
    // Update serial communication
}

void SerialCommunication::shutdown() {
    std::cout << "Shutting down Serial Communication..." << std::endl;
    pimpl_->status = "stopped";
}

} // namespace robotics
