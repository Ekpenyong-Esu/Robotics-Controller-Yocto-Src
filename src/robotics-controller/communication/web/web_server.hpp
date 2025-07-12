#pragma once

#include <memory>
#include <string>

namespace robotics {

/**
 * @brief HTTP web server for robot control interface
 *
 * Serves static files and provides REST API for robot control.
 */
class WebServer {
public:
    WebServer();
    ~WebServer();

    // Rule of five: delete copy, default move
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;
    WebServer(WebServer&&) noexcept = default;
    WebServer& operator=(WebServer&&) noexcept = default;

    bool initialize();
    void update();
    void shutdown();

    /**
     * @brief Get server status
     * @return Status string
     */
    std::string get_status() const;

    /**
     * @brief Update robot state for API responses
     * @param status Robot status string
     * @param left_speed Left motor speed
     * @param right_speed Right motor speed
     */
    void set_robot_state(const std::string& status, double left_speed, double right_speed);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
