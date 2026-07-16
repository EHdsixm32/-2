#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "core/detector.hpp"
#include "core/tracker.hpp"
#include "core/pid_controller.hpp"
#include "hardware/camera.hpp"
#include "hardware/serial_comm.hpp"
#include "utils/config_manager.hpp"
#include "utils/logger.hpp"

int main(int argc, char** argv) {
    Logger::init();
    auto& cfg = ConfigManager::getInstance();
    if (!cfg.load()) {
        LOG_ERROR("Failed to load config.yaml");
        return -1;
    }

    // 1. 初始化相机
    int cam_id = cfg.get<int>("camera.device_id", 0);
    int width = cfg.get<int>("camera.width", 640);
    int height = cfg.get<int>("camera.height", 480);
    Camera camera(cam_id, width, height);
    if (!camera.open()) {
        LOG_ERROR("Failed to open camera!");
        return -1;
    }
    float fx = cfg.get<float>("camera.focal_length_px", 600.0);
    camera.setFocalLength(fx);

    // 2. 初始化YOLO
    std::string model_path = cfg.get<std::string>("detector.model_path", "models/target.onnx");
    float conf_thresh = cfg.get<float>("detector.conf_threshold", 0.45);
    float nms_thresh = cfg.get<float>("detector.nms_threshold", 0.4);
    YOLODetector detector(model_path, conf_thresh, nms_thresh);

    // 3. 初始化跟踪器
    TargetTracker tracker;

    // 4. 初始化PID
    auto yaw_node = cfg.getNode("pid.yaw");
    auto pitch_node = cfg.getNode("pid.pitch");
    PIDParams yaw_params{yaw_node["kp"].as<double>(), yaw_node["ki"].as<double>(), 
                         yaw_node["kd"].as<double>(), yaw_node["integral_limit"].as<double>(),
                         yaw_node["output_limit"].as<double>()};
    PIDParams pitch_params{pitch_node["kp"].as<double>(), pitch_node["ki"].as<double>(),
                           pitch_node["kd"].as<double>(), pitch_node["integral_limit"].as<double>(),
                           pitch_node["output_limit"].as<double>()};
    PIDController pid_yaw(yaw_params);
    PIDController pid_pitch(pitch_params);

    // 5. 初始化串口
    std::string port = cfg.get<std::string>("serial.port", "/dev/ttyUSB0");
    int baud = cfg.get<int>("serial.baudrate", 115200);
    SerialComm serial(port, baud);
    if (!serial.open()) {
        LOG_WARN("Serial port open failed, running in simulation mode.");
    }

    // 6. 主循环
    cv::Mat frame;
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    LOG_INFO("Vision Tracker Started!");
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;
        if (dt > 0.1) dt = 0.033; // 防止卡顿时积分爆炸

        if (!camera.read(frame)) {
            LOG_WARN("Frame read failed!");
            continue;
        }
        if (frame.empty()) continue;

        // 检测
        auto detections = detector.detect(frame);
        Detection target;
        bool has_target = false;
        if (!detections.empty()) {
            // 取置信度最高的作为目标
            target = detections[0];
            has_target = true;
        }

        // 更新跟踪器
        if (has_target) {
            tracker.update(target, dt);
        } else {
            tracker.updateLost(dt);
        }

        cv::Point2f target_pos = tracker.getTargetPosition();
        TrackState state = tracker.getState();

        // 计算偏差 -> 角度 -> PID控制
        ControlCommand cmd{0, 0, false};
        if (state == TrackState::TRACKING && target_pos.x > 0 && target_pos.y > 0) {
            float cx = frame.cols / 2.0f;
            float cy = frame.rows / 2.0f;
            float dx = target_pos.x - cx;
            float dy = target_pos.y - cy;

            // 像素偏差转角度 (小角度近似)
            float angle_yaw = atan2(dx, fx);
            float angle_pitch = atan2(dy, fx);

            float yaw_ctrl = pid_yaw.update(angle_yaw, dt);
            float pitch_ctrl = pid_pitch.update(angle_pitch, dt);

            cmd.yaw_angle = yaw_ctrl;
            cmd.pitch_angle = pitch_ctrl;
            cmd.laser_enable = true;

            LOG_INFO("Frame {}: Target({:.1f},{:.1f}) | Yaw:{:.2f} Pitch:{:.2f}", 
                     frame_count++, target_pos.x, target_pos.y, yaw_ctrl, pitch_ctrl);
        } else if (state == TrackState::SEARCHING) {
            // 搜索模式下微动扫描，激光关闭
            cmd.yaw_angle = 2.0 * sin(current_time.time_since_epoch().count() * 1e-9);
            cmd.pitch_angle = 1.0 * cos(current_time.time_since_epoch().count() * 1e-9);
            cmd.laser_enable = false;
        } else {
            // LOST: 停止转动，关激光
            cmd.yaw_angle = 0;
            cmd.pitch_angle = 0;
            cmd.laser_enable = false;
        }

        // 发送串口指令
        if (serial.isOpen()) {
            serial.sendCommand(cmd);
        }

        // 可视化（调试）
        cv::Mat display = frame.clone();
        if (state == TrackState::TRACKING && target_pos.x > 0) {
            cv::circle(display, cv::Point(target_pos.x, target_pos.y), 5, cv::Scalar(0, 0, 255), -1);
        }
        cv::putText(display, "State: " + std::to_string(static_cast<int>(state)), 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0));
        cv::imshow("Tracker", display);
        if (cv::waitKey(1) == 'q') break;
    }

    serial.close();
    return 0;
}