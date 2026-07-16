// detector.cpp
#include "core/detector.hpp"
#include <spdlog/spdlog.h>

YOLODetector::YOLODetector(const std::string& model_path, float conf_thresh, float nms_thresh)
    : conf_threshold_(conf_thresh), nms_threshold_(nms_thresh) {
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        spdlog::info("YOLO model loaded: {}", model_path);
    } catch (const std::exception& e) {
        spdlog::error("Load model failed: {}", e.what());
    }
}

void YOLODetector::preprocess(const cv::Mat& frame, cv::Mat& blob) {
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0,
                           cv::Size(input_size_, input_size_),
                           cv::Scalar(), true, false);
}

std::vector<Detection> YOLODetector::postprocess(const std::vector<cv::Mat>& outputs,
                                                  const cv::Size& frame_size) {
    std::vector<Detection> detections;
    if (outputs.empty()) return detections;

    // 假设 outputs[0] 形状为 [1, num_attrs, num_boxes]
    cv::Mat output = outputs[0];
    int num_attrs = output.size[1];   // 通常 = 4 + num_classes
    int num_boxes = output.size[2];   // 检测框数量

    // 转置为 [num_boxes, num_attrs]
    output = output.reshape(1, num_boxes);   // 现在行数=num_boxes，列数=num_attrs

    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> class_ids;

    for (int i = 0; i < num_boxes; ++i) {
        float* ptr = output.ptr<float>(i);
        float cx = ptr[0];
        float cy = ptr[1];
        float w = ptr[2];
        float h = ptr[3];

        // 从第4维开始取类别分数（假设只有1类，取最大即可）
        float max_conf = 0;
        int max_id = 0;
        for (int j = 4; j < num_attrs; ++j) {
            if (ptr[j] > max_conf) {
                max_conf = ptr[j];
                max_id = j - 4;
            }
        }
        float conf = max_conf;
        if (conf < conf_threshold_) continue;

        // 映射到原图尺寸
        float x = (cx - w / 2) * frame_size.width;
        float y = (cy - h / 2) * frame_size.height;
        float width = w * frame_size.width;
        float height = h * frame_size.height;
        boxes.emplace_back(cv::Rect(x, y, width, height));
        confs.push_back(conf);
        class_ids.push_back(max_id);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, conf_threshold_, nms_threshold_, indices);

    for (int idx : indices) {
        Detection det;
        det.bbox = boxes[idx];
        det.confidence = confs[idx];
        det.class_id = class_ids[idx];
        det.center = cv::Point2f(boxes[idx].x + boxes[idx].width / 2.0,
                                 boxes[idx].y + boxes[idx].height / 2.0);
        detections.push_back(det);
    }
    return detections;
}

std::vector<Detection> YOLODetector::detect(const cv::Mat& frame) {
    cv::Mat blob;
    preprocess(frame, blob);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    return postprocess(outputs, frame.size());
}