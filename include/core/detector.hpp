#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct Detection {
    cv::Rect bbox;
    float confidence;
    int class_id;
    cv::Point2f center;
};

class YOLODetector {
public:
    YOLODetector(const std::string& model_path, float conf_thresh = 0.5, float nms_thresh = 0.4);
    ~YOLODetector();
    std::vector<Detection> detect(const cv::Mat& frame);

private:
    void* session_;  // 前向声明，避免头文件包含
    float conf_threshold_;
    float nms_threshold_;
    int input_width_ = 640;
    int input_height_ = 640;
    bool loaded_ = false;
    std::vector<std::string> class_names_;

    void preprocess(const cv::Mat& frame, std::vector<float>& blob, int& out_width, int& out_height);
    std::vector<Detection> postprocess(const std::vector<float>& output, const cv::Size& frame_size);
};