#pragma once

#include <memory>
#include <string>

namespace robotics {

class SerialCommunication {
public:
    SerialCommunication();
    ~SerialCommunication();

    // Delete copy constructor and copy assignment operator
    SerialCommunication(const SerialCommunication&) = delete;
    SerialCommunication& operator=(const SerialCommunication&) = delete;

    // Default move constructor and move assignment operator
    SerialCommunication(SerialCommunication&&) noexcept = default;
    SerialCommunication& operator=(SerialCommunication&&) noexcept = default;

    bool initialize();
    void update();
    void shutdown();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
