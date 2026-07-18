#include "utils/logger.hpp"

std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;

void Logger::init() {
    if (!logger_) {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/run.log", 1048576 * 5, 3);
        std::vector<spdlog::sink_ptr> sinks {console_sink, file_sink};
        logger_ = std::make_shared<spdlog::logger>("vision", sinks.begin(), sinks.end());
        logger_->set_level(spdlog::level::info);
        logger_->flush_on(spdlog::level::info);
    }
}