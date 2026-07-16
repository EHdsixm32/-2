#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
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
    std::vector<Detection> detect(const cv::Mat& frame);

private:
    cv::dnn::Net net_;
    float conf_threshold_;
    float nms_threshold_;
    int input_size_ = 640;
    std::vector<std::string> class_names_;

    void preprocess(const cv::Mat& frame, cv::Mat& blob);
    std::vector<Detection> postprocess(const std::vector<cv::Mat>& outputs, const cv::Size& frame_size);
};