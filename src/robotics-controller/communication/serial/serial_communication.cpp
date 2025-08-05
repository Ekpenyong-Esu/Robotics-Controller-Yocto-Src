#include "serial_communication.hpp" // Header for SerialCommunication class
#include <iostream>      // For logging and error output
#include <fcntl.h>        // For file control options
#include <termios.h>      // For serial port configuration
#include <unistd.h>       // For POSIX API (close, read, write)
#include <cstring>        // For string operations
#include <array>          // For fixed-size buffers

namespace robotics {
// SerialCommunication implementation for managing a serial port

// Private implementation struct for SerialCommunication
// Encapsulates all serial port state and logic
struct SerialCommunication::Impl {

    int serial_fd = -1;                  // Serial port file descriptor
    std::string device_path = "/dev/ttyUSB0"; // Default device path
    int baud_rate = 9600;                // Baud rate (communication speed)
    bool ready = false;                  // Port ready status
    std::string status = "stopped";     // Communication status

    // Communication buffers for RX and TX
    std::array<char, 256> rx_buffer;     // Receive buffer
    std::array<char, 256> tx_buffer;     // Transmit buffer


    // Destructor: closes the serial port if open
    ~Impl() {
        if (serial_fd >= 0) {
            close(serial_fd); // Ensure port is closed on destruction
        }
    }

    // Set up and configure the serial port
    // Returns true if successful, false otherwise
    bool setup_serial() {
        // Open serial port for reading and writing
        serial_fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd < 0) {
            std::cerr << "Failed to open serial device: " << device_path << std::endl;
            return false;
        }

        // Configure serial port settings
        struct termios options;
        tcgetattr(serial_fd, &options);

        // Set baud rate (speed of communication)
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

        // Configure for 8 data bits, no parity, 1 stop bit (8N1)
        options.c_cflag &= ~PARENB;   // No parity
        options.c_cflag &= ~CSTOPB;   // 1 stop bit
        options.c_cflag &= ~CSIZE;    // Clear size mask
        options.c_cflag |= CS8;       // 8 data bits
        options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode

        // Set raw input/output mode (no line processing)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
        options.c_oflag &= ~OPOST;                          // Raw output

        // Set read timeout: non-blocking, 0.1 second
        options.c_cc[VMIN] = 0;   // Non-blocking read
        options.c_cc[VTIME] = 1;  // 0.1 second timeout

        // Apply settings to the serial port
        if (tcsetattr(serial_fd, TCSANOW, &options) != 0) {
            std::cerr << "Failed to configure serial port" << std::endl;
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        return true;
    }
};

// Constructor: creates the implementation object
SerialCommunication::SerialCommunication() : pimpl_(std::make_unique<Impl>()) {}

// Destructor: cleans up resources
SerialCommunication::~SerialCommunication() = default;

// Initialize the serial communication (does not open the port yet)
// Sets status to running
bool SerialCommunication::initialize() {
    std::cout << "Initializing Serial Communication..." << std::endl;
    pimpl_->status = "running";
    return true;
}

// Update serial communication (placeholder for future logic)
void SerialCommunication::update() {
    // Update serial communication (not implemented)
}

// Shutdown the serial communication and clean up
// Sets status to stopped
void SerialCommunication::shutdown() {
    std::cout << "Shutting down Serial Communication..." << std::endl;
    pimpl_->status = "stopped";
}

} // namespace robotics
