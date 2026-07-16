#include "utils/config_manager.hpp"
#include <spdlog/spdlog.h>

ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::load(const std::string& path) {
    try {
        config_ = YAML::LoadFile(path);
        spdlog::info("Config loaded: {}", path);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Config load error: {}", e.what());
        return false;
    }
}