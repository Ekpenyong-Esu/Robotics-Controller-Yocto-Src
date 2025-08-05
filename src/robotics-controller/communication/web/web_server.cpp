#include "web_server.hpp" // WebServer class interface
#include <iostream>      // For logging and error output
#include <thread>        // For server and client threads
#include <sstream>       // For string stream operations
#include <fstream>       // For file serving
#include <cstring>       // For string operations
#include <sys/socket.h>  // For socket API
#include <netinet/in.h>  // For sockaddr_in
#include <unistd.h>      // For POSIX API (close, read, write)
#include <fcntl.h>       // For file control options

namespace robotics {
// Implementation of WebServer: HTTP server for robot control and status

// Private implementation struct for WebServer
struct WebServer::Impl {

    int server_socket = -1;         // Server socket file descriptor
    std::thread server_thread;      // Thread for server loop
    bool running = false;           // Server running flag
    bool ready = false;             // Server ready flag

    // Configuration
    int port = 8080;                // Listening port
    std::string web_root = "/usr/share/robotics-controller/www"; // Static file root
    std::string status = "stopped"; // Server status string

    // Robot state (for API endpoints)
    std::string robot_status = "idle"; // Robot status
    double left_motor_speed = 0.0;      // Left motor speed
    double right_motor_speed = 0.0;     // Right motor speed


    // Destructor: stops server and cleans up
    ~Impl() {
        std::cerr << "[WebServer::Impl] Destructor called" << std::endl;
        stop_server();
    }

    // Start the HTTP server in a background thread
    bool start_server() {
        // Create socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Failed to create server socket" << std::endl;
            return false;
        }

        // Set socket options (reuse address)
        int opt = 1;
        if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Failed to set socket options" << std::endl;
            close(server_socket);
            return false;
        }

        // Bind to port
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_socket, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind to port " << port << std::endl;
            close(server_socket);
            return false;
        }

        // Listen for connections
        if (listen(server_socket, 5) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_socket);
            return false;
        }

        running = true;
        // Start server loop in background thread
        server_thread = std::thread(&Impl::server_loop, this);

        std::cout << "Web server started on port " << port << std::endl;
        return true;
    }

    // Stop the HTTP server and clean up resources
    void stop_server() {
        std::cerr << "[WebServer::Impl] stop_server() called" << std::endl;
        running = false;
        // Closing the socket will unblock accept() in the server thread
        if (server_socket >= 0) {
            std::cerr << "[WebServer::Impl] Closing server socket..." << std::endl;
            close(server_socket);
            server_socket = -1;
        }
        // Join server thread if running
        if (server_thread.joinable()) {
            std::cerr << "[WebServer::Impl] Joining server thread..." << std::endl;
            try {
                server_thread.join();
            } catch (const std::system_error& e) {
                std::cerr << "[WebServer::Impl] Exception joining thread: " << e.what() << std::endl;
            }
        }
        std::cerr << "[WebServer::Impl] stop_server() complete" << std::endl;
    }

    // Main server loop: accepts client connections
    void server_loop() {
        while (running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
            if (client_socket < 0) {
                if (running) {
                    std::cerr << "Failed to accept client connection" << std::endl;
                }
                continue;
            }

            // Handle request in separate thread for basic concurrency
            std::thread(&Impl::handle_client, this, client_socket).detach();
        }
    }

    // Handle a single client connection
    void handle_client(int client_socket) {
        char buffer[1024] = {0};
        ssize_t bytes_read = read(client_socket, buffer, sizeof(buffer) - 1);

        if (bytes_read <= 0) {
            close(client_socket);
            return;
        }

        std::string request(buffer);           // Parse HTTP request
        std::string response = process_request(request); // Generate response

        write(client_socket, response.c_str(), response.length()); // Send response
        close(client_socket);                  // Close connection
    }

    // Parse HTTP request and route to appropriate handler
    std::string process_request(const std::string& request) {
        std::istringstream request_stream(request);
        std::string method;
        std::string path;
        std::string version;
        request_stream >> method >> path >> version;

        if (method == "GET") {
            // Serve static files or API endpoints
            if (path == "/" || path == "/index.html") {
                return serve_file("/index.html");
            } else if (path == "/api/status") {
                return serve_api_status();
            } else if (path == "/api/sensors") {
                return serve_api_sensors();
            } else if (path.find("/api/control/") == 0) {
                return handle_control_api(path);
            } else {
                return serve_file(path);
            }
        } else if (method == "POST") {
            // Handle POST API requests
            if (path.find("/api/") == 0) {
                return handle_post_api(path, request);
            }
        }

        // Unknown request: return 404
        return create_404_response();
    }

    // Serve a static file from web_root
    std::string serve_file(const std::string& path) {
        std::string file_path = web_root + path;
        std::ifstream file(file_path, std::ios::binary);

        if (!file.is_open()) {
            return create_404_response();
        }

        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

        std::string content_type = get_content_type(path);

        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: " << content_type << "\r\n";
        response << "Content-Length: " << content.length() << "\r\n";
        response << "Connection: close\r\n";
        response << "\r\n";
        response << content;

        return response.str();
    }

    // Serve robot status as JSON for /api/status
    std::string serve_api_status() {
        std::ostringstream json;
        json << "{\n";
        json << "  \"status\": \"" << robot_status << "\",\n";
        json << "  \"motors\": {\n";
        json << "    \"left\": " << left_motor_speed << ",\n";
        json << "    \"right\": " << right_motor_speed << "\n";
        json << "  },\n";
        json << "  \"timestamp\": " << std::time(nullptr) << "\n";
        json << "}";

        return create_json_response(json.str());
    }

    // Serve sensor data as JSON for /api/sensors
    std::string serve_api_sensors() {
        std::ostringstream json;
        json << "{\n";
        json << "  \"distance\": 25.4,\n";
        json << "  \"line_detected\": true,\n";
        json << "  \"line_position\": 0.1,\n";
        json << "  \"button_pressed\": false,\n";
        json << "  \"timestamp\": " << std::time(nullptr) << "\n";
        json << "}";

        return create_json_response(json.str());
    }

    // Handle control API endpoints (e.g., stop motors)
    std::string handle_control_api(const std::string& path) {
        if (path == "/api/control/stop") {
            left_motor_speed = 0.0;
            right_motor_speed = 0.0;
            robot_status = "stopped";
            return create_json_response("{\"result\": \"success\", \"action\": \"stop\"}");
        }

        return create_404_response();
    }

    // Handle POST API requests (not implemented)
    std::string handle_post_api(const std::string& path, const std::string& request) {
        return create_json_response("{\"result\": \"not_implemented\"}");
    }

    // Create HTTP response with JSON content
    std::string create_json_response(const std::string& json) {
        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: application/json\r\n";
        response << "Content-Length: " << json.length() << "\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "Connection: close\r\n";
        response << "\r\n";
        response << json;

        return response.str();
    }

    // Create HTTP 404 Not Found response
    std::string create_404_response() {
        std::string content = "404 Not Found";
        std::ostringstream response;
        response << "HTTP/1.1 404 Not Found\r\n";
        response << "Content-Type: text/plain\r\n";
        response << "Content-Length: " << content.length() << "\r\n";
        response << "Connection: close\r\n";
        response << "\r\n";
        response << content;

        return response.str();
    }

    // Determine content type for HTTP response based on file extension
    std::string get_content_type(const std::string& path) {
        size_t dot_pos = path.find_last_of('.');
        if (dot_pos != std::string::npos) {
            std::string ext = path.substr(dot_pos);
            if (ext == ".html") return "text/html";
            if (ext == ".css") return "text/css";
            if (ext == ".js") return "application/javascript";
            if (ext == ".json") return "application/json";
            if (ext == ".png") return "image/png";
            if (ext == ".jpg" || ext == ".jpeg") return "image/jpeg";
        }
        return "text/plain";
    }
};

// WebServer constructor: creates implementation object
WebServer::WebServer() : pimpl_(std::make_unique<Impl>()) {}

// WebServer destructor: cleans up resources
WebServer::~WebServer() = default;

// Initialize the web server (starts background thread)
bool WebServer::initialize() {
    std::cout << "Initializing Web Server..." << std::endl;

    if (!pimpl_->start_server()) {
        std::cerr << "Failed to start web server" << std::endl;
        return false;
    }

    pimpl_->ready = true;
    pimpl_->status = "running";
    return true;
}

// Update web server state (status string)
void WebServer::update() {
    // Web server runs in background thread
    // Update robot state for API responses
    if (pimpl_->ready) {
        pimpl_->status = "running on port " + std::to_string(pimpl_->port);
    }
}

// Shutdown the web server and clean up resources
void WebServer::shutdown() {
    std::cout << "Shutting down Web Server..." << std::endl;
    pimpl_->stop_server();
    pimpl_->ready = false;
    pimpl_->status = "stopped";
}

// Get current server status string
std::string WebServer::get_status() const {
    return pimpl_->status;
}

// Set robot state for API responses
void WebServer::set_robot_state(const std::string& status, double left_speed, double right_speed) {
    pimpl_->robot_status = status;
    pimpl_->left_motor_speed = left_speed;
    pimpl_->right_motor_speed = right_speed;
}

} // namespace robotics
