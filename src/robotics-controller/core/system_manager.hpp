#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>

namespace robotics {

/**
 * @brief Central system manager for the robotics controller
 *
 * The SystemManager coordinates all major subsystems and handles
 * system-wide configuration, initialization, and shutdown.
 */
class SystemManager {
public:
    SystemManager();
    ~SystemManager();

    /**
     * @brief Initialize the system manager
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Shutdown the system manager
     */
    void shutdown();

    /**
     * @brief Update system manager (called in main loop)
     */
    void update();

    /**
     * @brief Get system status
     * @return Current system status string
     */
    std::string get_status() const;

    /**
     * @brief Check if system is running
     * @return true if system is running normally
     */
    bool is_running() const;

    /**
     * @brief Get system configuration value
     * @param key Configuration key
     * @return Configuration value or empty string if not found
     */
    std::string get_config(const std::string& key) const;

    /**
     * @brief Set system configuration value
     * @param key Configuration key
     * @param value Configuration value
     */
    void set_config(const std::string& key, const std::string& value);

    /**
     * @brief Load configuration from file
     * @param config_file Path to configuration file
     * @return true if loaded successfully
     */
    bool load_config(const std::string& config_file);

    /**
     * @brief Save configuration to file
     * @param config_file Path to configuration file
     * @return true if saved successfully
     */
    bool save_config(const std::string& config_file);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
