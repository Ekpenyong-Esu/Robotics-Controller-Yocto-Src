#pragma once
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace robotics {

struct VisionData {
    bool line_detected = false;
    double line_angle = 0.0;
    double line_offset = 0.0;

    bool obstacle_detected = false;
    double obstacle_distance = 0.0;
    double obstacle_angle = 0.0;

    int frame_width = 0;
    int frame_height = 0;
    double fps = 0.0;
};

class VisionProcessor {
public:
    VisionProcessor();
    ~VisionProcessor();

    // Rule of five: delete copy and move (due to complex resource management)
    VisionProcessor(const VisionProcessor&) = delete;
    VisionProcessor& operator=(const VisionProcessor&) = delete;
    VisionProcessor(VisionProcessor&&) = delete;
    VisionProcessor& operator=(VisionProcessor&&) = delete;

    bool initialize();
    void update();
    void shutdown();
    std::string get_status() const;
    bool is_enabled() const;

    /**
     * @brief Enable or disable vision processing
     * @param enabled True to enable, false to disable
     */
    void set_enabled(bool enabled);

    /**
     * @brief Get the latest vision data
     * @return VisionData struct with processed information
     */
    VisionData get_vision_data() const;

    /**
     * @brief Get current frame as base64 encoded JPEG
     * @return Base64 encoded image string for web interface
     */
    std::string get_frame_base64() const;

    /**
     * @brief Set line detection parameters
     * @param hue_min Minimum hue value for line color
     * @param hue_max Maximum hue value for line color
     * @param sat_min Minimum saturation value
     * @param val_min Minimum value (brightness)
     */
    void set_line_detection_params(int hue_min, int hue_max, int sat_min, int val_min);

    /**
     * @brief Set obstacle detection parameters
     * @param min_contour_area Minimum area for obstacle detection
     * @param max_distance Maximum distance to consider as obstacle
     */
    void set_obstacle_detection_params(int min_contour_area, double max_distance);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace robotics
