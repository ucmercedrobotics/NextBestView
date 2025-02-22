#ifndef YOLO_DETECTOR_HPP
#define YOLO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct YoloResult {
    std::string class_name;
    float confidence;
    std::vector<float> bbox; // [x1, y1, x2, y2]
};

class YoloDetector {
public:
    YoloDetector(const std::string& model_path) {
        // Initialize YOLO (e.g., with libtorch or OpenCV DNN)
    }
    std::vector<YoloResult> detect(const cv::Mat& image) {
        // Placeholder implementation
        return std::vector<YoloResult>();
    }
};

#endif