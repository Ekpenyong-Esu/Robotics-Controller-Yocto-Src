#include "vision_processor.hpp"
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace robotics {

struct VisionProcessor::Impl {
    cv::VideoCapture cap;
    cv::Mat current_frame;
    cv::Mat processed_frame;

    bool ready = false;
    bool enabled = true;
    bool camera_opened = false;

    VisionData vision_data;

    // Camera settings
    int camera_id = 0;
    int frame_width = 640;
    int frame_height = 480;
    double target_fps = 30.0;

    // Line detection parameters (HSV color space)
    int line_hue_min = 0;     // For black line detection
    int line_hue_max = 180;
    int line_sat_min = 0;
    int line_sat_max = 255;
    int line_val_min = 0;     // Very dark values
    int line_val_max = 50;    // Low brightness for black

    // Obstacle detection parameters
    int obstacle_min_contour_area = 1000;
    double obstacle_max_distance = 100.0; // cm

    // Performance tracking
    std::chrono::steady_clock::time_point last_frame_time;
    double current_fps = 0.0;
    int frame_count = 0;

    // Region of interest for line detection (lower half of frame)
    cv::Rect line_roi;

    bool open_camera() {
        cap.open(camera_id);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera " << camera_id << std::endl;
            return false;
        }

        // Set camera properties
        cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
        cap.set(cv::CAP_PROP_FPS, target_fps);

        // Update actual properties
        frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

        // Set ROI for line detection (bottom half of frame)
        line_roi = cv::Rect(0, frame_height / 2, frame_width, frame_height / 2);

        vision_data.frame_width = frame_width;
        vision_data.frame_height = frame_height;

        camera_opened = true;
        std::cout << "Camera opened: " << frame_width << "x" << frame_height << " @ " << target_fps << " FPS" << std::endl;
        return true;
    }

    void detect_line(const cv::Mat& frame) {
        if (frame.empty()) return;

        // Extract ROI for line detection
        cv::Mat line_region = frame(line_roi);

        // Convert to HSV for better color detection
        cv::Mat hsv;
        cv::cvtColor(line_region, hsv, cv::COLOR_BGR2HSV);

        // Create mask for line color (black line)
        cv::Mat mask;
        cv::inRange(hsv,
                   cv::Scalar(line_hue_min, line_sat_min, line_val_min),
                   cv::Scalar(line_hue_max, line_sat_max, line_val_max),
                   mask);

        // Apply morphological operations to clean up the mask
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour (assumed to be the line)
            auto largest_contour = *std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            if (cv::contourArea(largest_contour) > 100) { // Minimum area threshold
                vision_data.line_detected = true;

                // Fit line to contour
                cv::Vec4f line_params;
                cv::fitLine(largest_contour, line_params, cv::DIST_L2, 0, 0.01, 0.01);

                // Calculate line angle (in degrees)
                vision_data.line_angle = std::atan2(line_params[1], line_params[0]) * 180.0 / M_PI;

                // Calculate offset from center
                cv::Point2f line_center(line_params[2], line_params[3]);
                double frame_center_x = line_region.cols / 2.0;
                vision_data.line_offset = (line_center.x - frame_center_x) / frame_center_x; // Normalized -1 to 1

                // Draw line on processed frame for visualization
                cv::Point pt1, pt2;
                pt1.x = line_center.x - 100 * line_params[0];
                pt1.y = line_center.y - 100 * line_params[1];
                pt2.x = line_center.x + 100 * line_params[0];
                pt2.y = line_center.y + 100 * line_params[1];

                cv::line(processed_frame(line_roi), pt1, pt2, cv::Scalar(0, 255, 0), 3);
                cv::circle(processed_frame(line_roi), line_center, 5, cv::Scalar(0, 0, 255), -1);
            } else {
                vision_data.line_detected = false;
            }
        } else {
            vision_data.line_detected = false;
        }
    }

    void detect_obstacles(const cv::Mat& frame) {
        if (frame.empty()) return;

        // Use upper half of frame for obstacle detection
        cv::Rect obstacle_roi(0, 0, frame_width, frame_height / 2);
        cv::Mat obstacle_region = frame(obstacle_roi);

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(obstacle_region, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Edge detection
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        vision_data.obstacle_detected = false;
        double largest_area = 0;
        cv::Point2f obstacle_center;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > obstacle_min_contour_area && area > largest_area) {
                largest_area = area;
                vision_data.obstacle_detected = true;

                // Calculate obstacle center
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 > 0) {
                    obstacle_center.x = moments.m10 / moments.m00;
                    obstacle_center.y = moments.m01 / moments.m00;

                    // Calculate angle from center
                    double frame_center_x = obstacle_region.cols / 2.0;
                    vision_data.obstacle_angle = std::atan2(obstacle_center.x - frame_center_x, obstacle_region.rows - obstacle_center.y) * 180.0 / M_PI;

                    // Rough distance estimation based on contour size
                    vision_data.obstacle_distance = std::max(10.0, obstacle_max_distance * (1000.0 / area));

                    // Draw obstacle on processed frame
                    cv::drawContours(processed_frame(obstacle_roi), std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2);
                    cv::circle(processed_frame(obstacle_roi), obstacle_center, 10, cv::Scalar(0, 255, 255), -1);
                }
            }
        }
    }

    void update_fps() {
        auto current_time = std::chrono::steady_clock::now();
        if (frame_count > 0) {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_frame_time);
            if (duration.count() > 0) {
                current_fps = 1000.0 / duration.count();
                vision_data.fps = current_fps;
            }
        }
        last_frame_time = current_time;
        frame_count++;
    }
};

VisionProcessor::VisionProcessor() : pimpl_(std::make_unique<Impl>()) {}

VisionProcessor::~VisionProcessor() = default;

bool VisionProcessor::initialize() {
    std::cout << "Initializing Vision Processor with OpenCV..." << std::endl;

    if (!pimpl_->open_camera()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        pimpl_->enabled = false;
        return false;
    }

    pimpl_->ready = true;
    std::cout << "Vision Processor initialized successfully" << std::endl;
    return true;
}

void VisionProcessor::update() {
    if (!pimpl_->ready || !pimpl_->enabled || !pimpl_->camera_opened) {
        return;
    }

    // Capture frame
    if (!pimpl_->cap.read(pimpl_->current_frame)) {
        std::cerr << "Failed to capture frame" << std::endl;
        return;
    }

    if (pimpl_->current_frame.empty()) {
        return;
    }

    // Copy frame for processing
    pimpl_->current_frame.copyTo(pimpl_->processed_frame);

    // Perform line detection
    pimpl_->detect_line(pimpl_->current_frame);

    // Perform obstacle detection
    pimpl_->detect_obstacles(pimpl_->current_frame);

    // Update FPS counter
    pimpl_->update_fps();
}

void VisionProcessor::shutdown() {
    std::cout << "Shutting down Vision Processor..." << std::endl;
    pimpl_->ready = false;
    pimpl_->enabled = false;

    if (pimpl_->cap.isOpened()) {
        pimpl_->cap.release();
    }

    pimpl_->camera_opened = false;
}

std::string VisionProcessor::get_status() const {
    if (!pimpl_->ready) {
        return "not_ready";
    }
    if (!pimpl_->enabled) {
        return "disabled";
    }
    if (!pimpl_->camera_opened) {
        return "camera_error";
    }

    return "active - FPS: " + std::to_string(static_cast<int>(pimpl_->vision_data.fps)) +
           ", Line: " + (pimpl_->vision_data.line_detected ? "YES" : "NO") +
           ", Obstacle: " + (pimpl_->vision_data.obstacle_detected ? "YES" : "NO");
}

bool VisionProcessor::is_enabled() const {
    return pimpl_->enabled;
}

void VisionProcessor::set_enabled(bool enabled) {
    pimpl_->enabled = enabled;
}

VisionData VisionProcessor::get_vision_data() const {
    return pimpl_->vision_data;
}

std::string VisionProcessor::get_frame_base64() const {
    if (pimpl_->processed_frame.empty()) {
        return "";
    }

    // Encode frame as JPEG
    std::vector<uchar> buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 80};

    if (!cv::imencode(".jpg", pimpl_->processed_frame, buffer, encode_params)) {
        return "";
    }

    // Convert to base64
    static const char base64_chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string encoded;
    int val = 0, bits = -6;
    for (unsigned char c : buffer) {
        val = (val << 8) + c;
        bits += 8;
        while (bits >= 0) {
            encoded.push_back(base64_chars[(val >> bits) & 0x3F]);
            bits -= 6;
        }
    }
    if (bits > -6) {
        encoded.push_back(base64_chars[((val << 8) >> (bits + 8)) & 0x3F]);
    }
    while (encoded.size() % 4) {
        encoded.push_back('=');
    }

    return encoded;
}

void VisionProcessor::set_line_detection_params(int hue_min, int hue_max, int sat_min, int val_min) {
    pimpl_->line_hue_min = hue_min;
    pimpl_->line_hue_max = hue_max;
    pimpl_->line_sat_min = sat_min;
    pimpl_->line_val_min = val_min;
}

void VisionProcessor::set_obstacle_detection_params(int min_contour_area, double max_distance) {
    pimpl_->obstacle_min_contour_area = min_contour_area;
    pimpl_->obstacle_max_distance = max_distance;
}

} // namespace robotics
