#include "core/detector.hpp"
#include <spdlog/spdlog.h>
#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cstring>
#include <memory>

class ONNXSession {
public:
    Ort::Session session{nullptr};
    Ort::MemoryInfo memory_info{nullptr};
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
    bool loaded = false;
};

YOLODetector::YOLODetector(const std::string& model_path, float conf_thresh, float nms_thresh)
    : conf_threshold_(conf_thresh), nms_threshold_(nms_thresh), session_(nullptr) {
    try {
        // 使用静态 Env，避免重复初始化
        static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolo");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        auto session_ptr = new ONNXSession();
        session_ptr->session = Ort::Session(env, model_path.c_str(), session_options);
        session_ptr->memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        session_ptr->loaded = true;

        Ort::AllocatorWithDefaultOptions allocator;
        auto input_info = session_ptr->session.GetInputTypeInfo(0);
        auto input_shape_info = input_info.GetTensorTypeAndShapeInfo();
        session_ptr->input_shape = input_shape_info.GetShape();

        auto output_info = session_ptr->session.GetOutputTypeInfo(0);
        auto output_shape_info = output_info.GetTensorTypeAndShapeInfo();
        session_ptr->output_shape = output_shape_info.GetShape();

        if (session_ptr->input_shape.size() >= 4)
            spdlog::info("ONNX input shape: {}x{}x{}", session_ptr->input_shape[1], session_ptr->input_shape[2], session_ptr->input_shape[3]);
        if (session_ptr->output_shape.size() >= 3)
            spdlog::info("ONNX output shape: {}x{}x{}", session_ptr->output_shape[0], session_ptr->output_shape[1], session_ptr->output_shape[2]);

        session_ = session_ptr;
        loaded_ = true;
        spdlog::info("YOLO model loaded via ONNXRuntime: {}", model_path);
    } catch (const std::exception& e) {
        spdlog::error("ONNXRuntime load failed: {}", e.what());
        loaded_ = false;
        if (session_) {
            delete static_cast<ONNXSession*>(session_);
            session_ = nullptr;
        }
    }
}

YOLODetector::~YOLODetector() {
    if (session_) {
        delete static_cast<ONNXSession*>(session_);
        session_ = nullptr;
    }
}

void YOLODetector::preprocess(const cv::Mat& frame, std::vector<float>& blob, int& out_width, int& out_height) {
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(input_width_, input_height_));
    resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);
    std::vector<cv::Mat> channels(3);
    cv::split(resized, channels);
    blob.clear();
    blob.reserve(input_width_ * input_height_ * 3);
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < input_height_; ++r) {
            const float* row = channels[c].ptr<float>(r);
            blob.insert(blob.end(), row, row + input_width_);
        }
    }
    out_width = input_width_;
    out_height = input_height_;
}

std::vector<Detection> YOLODetector::postprocess(const std::vector<float>& output, const cv::Size& frame_size) {
    std::vector<Detection> detections;
    if (output.empty()) return detections;

    auto* sess = static_cast<ONNXSession*>(session_);
    if (!sess || !sess->loaded) return detections;

    size_t total = output.size();
    int num_attrs = 5;
    if (total % 84 == 0) num_attrs = 84;
    else if (total % 5 == 0) num_attrs = 5;
    else {
        spdlog::error("Cannot determine output format from total elements {}", total);
        return detections;
    }
    int num_boxes = static_cast<int>(total / num_attrs);
    spdlog::info("Detected {} boxes, each with {} attributes", num_boxes, num_attrs);
    if (num_boxes == 0) return detections;

    // 转置
    std::vector<float> transposed(total);
    for (int b = 0; b < num_boxes; ++b) {
        for (int a = 0; a < num_attrs; ++a) {
            transposed[b * num_attrs + a] = output[a * num_boxes + b];
        }
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> class_ids;

    for (int i = 0; i < num_boxes; ++i) {
        const float* row = transposed.data() + i * num_attrs;
        float cx = row[0];
        float cy = row[1];
        float w = row[2];
        float h = row[3];

        float max_conf = 0.0f;
        int max_id = 0;
        for (int j = 4; j < num_attrs; ++j) {
            float conf = row[j];
            if (conf > max_conf) {
                max_conf = conf;
                max_id = j - 4;
            }
        }
        if (max_conf < conf_threshold_) continue;

        float x = (cx - w / 2) * frame_size.width;
        float y = (cy - h / 2) * frame_size.height;
        float width = w * frame_size.width;
        float height = h * frame_size.height;
        boxes.emplace_back(cv::Rect(static_cast<int>(x), static_cast<int>(y),
                                    static_cast<int>(width), static_cast<int>(height)));
        confs.push_back(max_conf);
        class_ids.push_back(max_id);
    }

    if (!boxes.empty()) {
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
    }
    return detections;
}

std::vector<Detection> YOLODetector::detect(const cv::Mat& frame) {
    if (!loaded_ || !session_) return {};

    auto* sess = static_cast<ONNXSession*>(session_);
    if (!sess || !sess->loaded) return {};

    try {
        // 1. 预处理
        std::vector<float> input_blob;
        int in_w, in_h;
        preprocess(frame, input_blob, in_w, in_h);

        // 2. 创建输入 Tensor（使用 std::vector 确保数据生命周期）
        std::vector<int64_t> input_shape = {1, 3, in_h, in_w};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            sess->memory_info, input_blob.data(), input_blob.size(),
            input_shape.data(), input_shape.size());

        // 3. 准备输入/输出名称（确保指针在 Run 调用期间有效）
        auto input_names_vec = sess->session.GetInputNames();
        std::vector<const char*> input_names;
        input_names.reserve(input_names_vec.size());
        for (const auto& name : input_names_vec) {
            input_names.push_back(name.c_str());
        }

        auto output_names_vec = sess->session.GetOutputNames();
        std::vector<const char*> output_names;
        output_names.reserve(output_names_vec.size());
        for (const auto& name : output_names_vec) {
            output_names.push_back(name.c_str());
        }

        // 4. 运行推理
        std::vector<Ort::Value> output_values;
        Ort::RunOptions run_options;
        output_values = sess->session.Run(run_options,
                                          input_names.data(), &input_tensor, 1,
                                          output_names.data(), output_names.size());

        // 5. 提取输出
        if (output_values.empty()) {
            spdlog::warn("No output from ONNXRuntime");
            return {};
        }

        auto& output_tensor = output_values[0];
        auto type_info = output_tensor.GetTensorTypeAndShapeInfo();
        size_t num_elements = type_info.GetElementCount();
        if (num_elements == 0) {
            spdlog::warn("Output tensor has zero elements");
            return {};
        }

        float* output_data = output_tensor.GetTensorMutableData<float>();
        if (!output_data) {
            spdlog::error("Output data is null");
            return {};
        }

        std::vector<float> output_vec(output_data, output_data + num_elements);

        // 6. 后处理
        return postprocess(output_vec, frame.size());

    } catch (const std::exception& e) {
        spdlog::error("ONNXRuntime inference failed: {}", e.what());
        return {};
    }
}