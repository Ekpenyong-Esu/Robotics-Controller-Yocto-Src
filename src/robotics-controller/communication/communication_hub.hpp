#pragma once
#include <memory>
#include <string>

namespace robotics {

class CommunicationHub {
public:
    CommunicationHub();
    ~CommunicationHub();
    bool initialize();
    void update();
    void shutdown();
    std::string get_status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
