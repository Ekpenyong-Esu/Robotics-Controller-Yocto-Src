#pragma once // Ensure single inclusion

#include <memory>   // For std::unique_ptr
#include <string>   // For device path and status

namespace robotics {
// SerialCommunication provides an interface for serial port management

class SerialCommunication {
public:
    SerialCommunication();   // Constructor
    ~SerialCommunication();  // Destructor

    // Non-copyable, movable
    SerialCommunication(const SerialCommunication&) = delete;
    SerialCommunication& operator=(const SerialCommunication&) = delete;
    SerialCommunication(SerialCommunication&&) noexcept = default;
    SerialCommunication& operator=(SerialCommunication&&) noexcept = default;

    // Initialize serial communication (does not open port)
    bool initialize();
    // Update communication (placeholder)
    void update();
    // Shutdown communication
    void shutdown();

private:
    struct Impl;                    // Private implementation
    std::unique_ptr<Impl> pimpl_;   // Pointer to implementation
};

} // namespace robotics
