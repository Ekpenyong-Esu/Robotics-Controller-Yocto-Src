#pragma once // Ensure single inclusion

#include <memory>   // For std::unique_ptr
#include <string>   // For status and API data

namespace robotics {
// WebServer provides an HTTP interface for robot control and status

/**
 * @brief HTTP web server for robot control interface
 *
 * Serves static files and provides REST API for robot control.
 */
class WebServer {
public:
    WebServer();   // Constructor
    ~WebServer();  // Destructor

    // Non-copyable, movable
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;
    WebServer(WebServer&&) noexcept = default;
    WebServer& operator=(WebServer&&) noexcept = default;

    // Initialize the web server (setup resources, does not start serving)
    bool initialize();
    // Update server state (placeholder for future logic)
    void update();
    // Shutdown the web server and clean up resources
    void shutdown();

    // Get current server status string
    std::string get_status() const;

    // Set robot state for API responses
    // status: robot status string
    // left_speed, right_speed: motor speeds
    void set_robot_state(const std::string& status, double left_speed, double right_speed);

private:
    struct Impl;                    // Private implementation
    std::unique_ptr<Impl> pimpl_;   // Pointer to implementation
};

} // namespace robotics
