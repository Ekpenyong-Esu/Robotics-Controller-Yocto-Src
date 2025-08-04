#include "web_server.hpp"
#include <iostream>
#include <thread>
#include <sstream>
#include <fstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

namespace robotics {

struct WebServer::Impl {
    int server_socket = -1;
    std::thread server_thread;
    bool running = false;
    bool ready = false;

    // Configuration
    int port = 8080;
    std::string web_root = "/usr/share/robotics-controller/www";
    std::string status = "stopped";

    // Robot state (for API endpoints)
    std::string robot_status = "idle";
    double left_motor_speed = 0.0;
    double right_motor_speed = 0.0;

    ~Impl() {
        std::cerr << "[WebServer::Impl] Destructor called" << std::endl;
        stop_server();
    }

    bool start_server() {
        // Create socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Failed to create server socket" << std::endl;
            return false;
        }

        // Set socket options
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
        server_thread = std::thread(&Impl::server_loop, this);

        std::cout << "Web server started on port " << port << std::endl;
        return true;
    }

    void stop_server() {
        std::cerr << "[WebServer::Impl] stop_server() called" << std::endl;
        running = false;
        // Closing the socket will unblock accept() in the server thread
        if (server_socket >= 0) {
            std::cerr << "[WebServer::Impl] Closing server socket..." << std::endl;
            close(server_socket);
            server_socket = -1;
        }
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

    void handle_client(int client_socket) {
        char buffer[1024] = {0};
        ssize_t bytes_read = read(client_socket, buffer, sizeof(buffer) - 1);

        if (bytes_read <= 0) {
            close(client_socket);
            return;
        }

        std::string request(buffer);
        std::string response = process_request(request);

        write(client_socket, response.c_str(), response.length());
        close(client_socket);
    }

    std::string process_request(const std::string& request) {
        std::istringstream request_stream(request);
        std::string method;
        std::string path;
        std::string version;
        request_stream >> method >> path >> version;

        if (method == "GET") {
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
            if (path.find("/api/") == 0) {
                return handle_post_api(path, request);
            }
        }

        return create_404_response();
    }

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

    std::string handle_control_api(const std::string& path) {
        // Simple control API endpoints
        if (path == "/api/control/stop") {
            left_motor_speed = 0.0;
            right_motor_speed = 0.0;
            robot_status = "stopped";
            return create_json_response("{\"result\": \"success\", \"action\": \"stop\"}");
        }

        return create_404_response();
    }

    std::string handle_post_api(const std::string& path, const std::string& request) {
        // Handle POST requests for motor control, etc.
        return create_json_response("{\"result\": \"not_implemented\"}");
    }

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

WebServer::WebServer() : pimpl_(std::make_unique<Impl>()) {}

WebServer::~WebServer() = default;

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

void WebServer::update() {
    // Web server runs in background thread
    // Update robot state for API responses
    if (pimpl_->ready) {
        pimpl_->status = "running on port " + std::to_string(pimpl_->port);
    }
}

void WebServer::shutdown() {
    std::cout << "Shutting down Web Server..." << std::endl;
    pimpl_->stop_server();
    pimpl_->ready = false;
    pimpl_->status = "stopped";
}

std::string WebServer::get_status() const {
    return pimpl_->status;
}

void WebServer::set_robot_state(const std::string& status, double left_speed, double right_speed) {
    pimpl_->robot_status = status;
    pimpl_->left_motor_speed = left_speed;
    pimpl_->right_motor_speed = right_speed;
}

} // namespace robotics
