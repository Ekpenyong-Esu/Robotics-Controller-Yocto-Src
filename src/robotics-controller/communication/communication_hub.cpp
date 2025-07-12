#include "communication_hub.hpp"
#include "serial/serial_communication.hpp"
#include "web/web_server.hpp"
#include <iostream>

namespace robotics {

struct CommunicationHub::Impl {
    bool ready = false;
    std::unique_ptr<SerialCommunication> serial;
    std::unique_ptr<WebServer> web_server;
};

CommunicationHub::CommunicationHub() : pimpl_(std::make_unique<Impl>()) {
    pimpl_->serial = std::make_unique<SerialCommunication>();
    pimpl_->web_server = std::make_unique<WebServer>();
}

CommunicationHub::~CommunicationHub() = default;

bool CommunicationHub::initialize() {
    std::cout << "Initializing Communication Hub..." << std::endl;

    bool success = true;
    success &= pimpl_->serial->initialize();
    success &= pimpl_->web_server->initialize();

    pimpl_->ready = success;
    return success;
}

void CommunicationHub::update() {
    if (pimpl_->ready) {
        pimpl_->serial->update();
        pimpl_->web_server->update();
    }
}

void CommunicationHub::shutdown() {
    std::cout << "Shutting down Communication Hub..." << std::endl;
    if (pimpl_->serial) pimpl_->serial->shutdown();
    if (pimpl_->web_server) pimpl_->web_server->shutdown();
    pimpl_->ready = false;
}

std::string CommunicationHub::get_status() const { return pimpl_->ready ? "ready" : "stopped"; }

} // namespace robotics
