#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

// Reading a type file (see TypeCatalog.h). Every accessor names the file
// and key when a value is absent or the wrong kind, so a typo in a type
// file fails at load with a message that says where. Include only from a
// catalog's .cpp: this pulls in json.hpp.
namespace TypeFileJson {

inline nlohmann::json Read(const std::filesystem::path& file, const std::string& catalogName) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error(catalogName + ": cannot open " + file.string());
    try {
        return nlohmann::json::parse(in);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(catalogName + ": " + file.string() + " is not valid JSON: " + e.what());
    }
}

inline double RequireNumber(const nlohmann::json& block, const char* key, const std::string& where) {
    if (!block.contains(key) || !block.at(key).is_number()) {
        throw std::runtime_error(where + ": missing or non-numeric \"" + key + "\"");
    }
    return block.at(key).get<double>();
}

inline std::string RequireString(const nlohmann::json& block, const char* key, const std::string& where) {
    if (!block.contains(key) || !block.at(key).is_string()) {
        throw std::runtime_error(where + ": missing or non-string \"" + key + "\"");
    }
    return block.at(key).get<std::string>();
}

inline bool RequireBool(const nlohmann::json& block, const char* key, const std::string& where) {
    if (!block.contains(key) || !block.at(key).is_boolean()) {
        throw std::runtime_error(where + ": missing or non-boolean \"" + key + "\"");
    }
    return block.at(key).get<bool>();
}

inline const nlohmann::json& RequireObject(const nlohmann::json& block, const char* key, const std::string& where) {
    if (!block.contains(key) || !block.at(key).is_object()) {
        throw std::runtime_error(where + ": missing \"" + key + "\" block");
    }
    return block.at(key);
}

}  // namespace TypeFileJson
