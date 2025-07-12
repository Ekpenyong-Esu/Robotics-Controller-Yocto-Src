#pragma once

#include <memory>
#include <string>

namespace robotics {

class SerialCommunication {
public:
    SerialCommunication();
    ~SerialCommunication();

    bool initialize();
    void update();
    void shutdown();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
