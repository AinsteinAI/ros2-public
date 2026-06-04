#include "rclcpp/rclcpp.hpp"
#include "radar_muniu/radar_parser.hpp"
#include "rcutils/filesystem.h"
#include <vector>
#include <memory>
#include <cstdint>
#include <string>
#include <iostream>
#include "rclcpp/parameter_map.hpp"

using namespace radar_muniu;

// Extract parameter parsing logic into an independent function
std::vector<RadarConfig> loadRadarParameters(rclcpp::Node* node) {
    std::vector<RadarConfig> radar_configs;

    try {
        // Declare parameters for Radar 0
        node->declare_parameter<std::string>("radar0.radar_id", "");
        node->declare_parameter<std::string>("radar0.frame_id", "");
        node->declare_parameter<std::string>("radar0.parent_frame_id", "");
        node->declare_parameter<std::string>("radar0.protocol", "");
        node->declare_parameter<std::string>("radar0.local_ip", "");
        node->declare_parameter<std::uint16_t>("radar0.local_port", 0);
        node->declare_parameter<std::string>("radar0.remote_ip", "");
        node->declare_parameter<std::uint16_t>("radar0.remote_port", 0);
        node->declare_parameter<std::vector<int64_t>>("radar0.service", std::vector<int64_t>());
        node->declare_parameter<std::string>("radar0.vsomeip_config", "");
        node->declare_parameter<std::float_t>("radar0.install_x", 0);
        node->declare_parameter<std::float_t>("radar0.install_y", 0);
        node->declare_parameter<std::float_t>("radar0.install_z", 0);
        node->declare_parameter<std::float_t>("radar0.install_roll", 0);
        node->declare_parameter<std::float_t>("radar0.install_pitch", 0);
        node->declare_parameter<std::float_t>("radar0.install_yaw", 0);
        node->declare_parameter<std::string>("radar0.direct", "");
        

        // Declare parameters for Radar 1
        node->declare_parameter<std::string>("radar1.radar_id", "");
        node->declare_parameter<std::string>("radar1.frame_id", "");
        node->declare_parameter<std::string>("radar1.parent_frame_id", "");
        node->declare_parameter<std::string>("radar1.protocol", "");
        node->declare_parameter<std::string>("radar1.device", "");
        node->declare_parameter<std::uint16_t>("radar1.arb_bitrate", 0);
        node->declare_parameter<std::uint16_t>("radar1.data_bitrate", 0);
        node->declare_parameter<std::uint8_t>("radar1.channel", 0);
        node->declare_parameter<std::string>("radar1.dbc_file", "");
        node->declare_parameter<std::float_t>("radar1.install_x", 0);
        node->declare_parameter<std::float_t>("radar1.install_y", 0);
        node->declare_parameter<std::float_t>("radar1.install_z", 0);
        node->declare_parameter<std::float_t>("radar1.install_roll", 0);
        node->declare_parameter<std::float_t>("radar1.install_pitch", 0);
        node->declare_parameter<std::float_t>("radar1.install_yaw", 0);
        node->declare_parameter<std::string>("radar1.direct", "");
        node->declare_parameter<std::vector<std::int64_t>>("radar1.parse_rule.begin_message", std::vector<std::int64_t>());
        node->declare_parameter<std::vector<std::int64_t>>("radar1.parse_rule.end_message", std::vector<std::int64_t>());
        node->declare_parameter<std::vector<std::int64_t>>("radar1.parse_rule.object_message", std::vector<std::int64_t>());
        node->declare_parameter<std::string>("radar1.parse_rule.id_signal", "");
        node->declare_parameter<std::string>("radar1.parse_rule.x_signal", "");
        node->declare_parameter<std::string>("radar1.parse_rule.y_signal", "");
        node->declare_parameter<std::string>("radar1.parse_rule.exist_signal", "");
        node->declare_parameter<std::string>("radar1.parse_rule.valid_signal", "");
        node->declare_parameter<std::uint8_t>("radar1.parse_rule.obj_idx_pos", 0);
        


        // Declare parameters for Radar 2
        node->declare_parameter<std::string>("radar2.radar_id", "");
        node->declare_parameter<std::string>("radar2.frame_id", "");
        node->declare_parameter<std::string>("radar2.parent_frame_id", "");
        node->declare_parameter<std::string>("radar2.protocol", "");
        node->declare_parameter<std::string>("radar2.device", "");
        node->declare_parameter<std::uint16_t>("radar2.arb_bitrate", 0);
        node->declare_parameter<std::uint16_t>("radar2.data_bitrate", 0);
        node->declare_parameter<std::uint8_t>("radar2.channel", 0);
        node->declare_parameter<std::string>("radar2.dbc_file", "");
        node->declare_parameter<std::float_t>("radar2.install_x", 0);
        node->declare_parameter<std::float_t>("radar2.install_y", 0);
        node->declare_parameter<std::float_t>("radar2.install_z", 0);
        node->declare_parameter<std::float_t>("radar2.install_roll", 0);
        node->declare_parameter<std::float_t>("radar2.install_pitch", 0);
        node->declare_parameter<std::float_t>("radar2.install_yaw", 0);
        node->declare_parameter<std::string>("radar2.direct", "");
        node->declare_parameter<std::vector<std::int64_t>>("radar2.parse_rule.begin_message", std::vector<std::int64_t>());
        node->declare_parameter<std::vector<std::int64_t>>("radar2.parse_rule.end_message", std::vector<std::int64_t>());
        node->declare_parameter<std::vector<std::int64_t>>("radar2.parse_rule.object_message", std::vector<std::int64_t>());
        node->declare_parameter<std::string>("radar2.parse_rule.id_signal", "");
        node->declare_parameter<std::string>("radar2.parse_rule.x_signal", "");
        node->declare_parameter<std::string>("radar2.parse_rule.y_signal", "");
        node->declare_parameter<std::string>("radar2.parse_rule.exist_signal", "");
        node->declare_parameter<std::string>("radar2.parse_rule.valid_signal", "");
        node->declare_parameter<std::uint8_t>("radar2.parse_rule.obj_idx_pos", 0);

        // Get config file path parameter
        std::string config_path;
        node->get_parameter("config_path", config_path);

        // Parse YAML file into parameter map
        rclcpp::ParameterMap param_map = rclcpp::parameter_map_from_yaml_file(config_path);
        for (const auto& node_param_pair : param_map) {
            const std::string& node_name = node_param_pair.first;
            const std::vector<rclcpp::Parameter>& parameters = node_param_pair.second;
            std::cout << "===== node: " << node_name << " =====" << std::endl;
            for (const rclcpp::Parameter& param : parameters) {
                std::cout << "name: " << param.get_name() << std::endl;
                std::cout << "type: " << param.get_type_name() << std::endl;
                std::cout << "value: " << param.value_to_string() << std::endl;
                std::cout << "-------------------------" << std::endl;
            }
        }

        // Set parameters from YAML to current node
        auto node_params_it = param_map.find(std::string("/") + node->get_name());
        if (node_params_it == param_map.end()) {
            RCLCPP_WARN(node->get_logger(), "No parameters found for node '%s' in YAML file.", node->get_name());
        } else {
            node->set_parameters(node_params_it->second);
            RCLCPP_INFO(node->get_logger(), "Successfully loaded %zu parameters from YAML file for node '%s'.",
                        node_params_it->second.size(), node->get_name());
        }

        // Dynamically get the number of radars
        size_t count = 0;
        std::vector<size_t> radar_id_list;
        while (true) {
            std::string radar_id_param = "radar" + std::to_string(count) + ".radar_id";
            std::string radar_id;
            if (node->has_parameter(radar_id_param)) {
                if (node->get_parameter(radar_id_param, radar_id) && !radar_id.empty()) {
                    radar_id_list.push_back(count);
                }
                count++;
            }
            else {
                break;
            }
        }

        // Parse configuration parameters for each radar
        for (size_t i = 0; i < radar_id_list.size(); ++i) {
            std::string prefix = "radar" + std::to_string(radar_id_list.at(i));
            if (!node->has_parameter(prefix + ".radar_id")) {
                continue;
            }

            RadarConfig config;
            // Extract basic parameters
            try {
                auto ret = node->get_parameter(prefix + ".radar_id", config.radar_id);
                std::cout << "config radar_id: " << config.radar_id << std::endl;
                if (!ret) {
                    continue;
                }
                node->get_parameter(prefix + ".frame_id", config.frame_id);
                node->get_parameter(prefix + ".parent_frame_id", config.parent_frame_id);
                node->get_parameter(prefix + ".protocol", config.protocol);
                node->get_parameter(prefix + ".install_x", config.x);
                node->get_parameter(prefix + ".install_y", config.y);
                node->get_parameter(prefix + ".install_z", config.z);
                node->get_parameter(prefix + ".install_roll", config.roll);
                node->get_parameter(prefix + ".install_pitch", config.pitch);
                node->get_parameter(prefix + ".install_yaw", config.yaw);
                node->get_parameter(prefix + ".direct", config.direct);


                std::cout << "config protocol: " << config.protocol << std::endl;

            } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
                RCLCPP_WARN(node->get_logger(), "Radar %zu missing basic config: %s, skipping", i, e.what());
                continue;
            }

            // Extract protocol-related configurations
            if (config.protocol == "udp") {
                try {
                    node->get_parameter(prefix + ".local_ip", config.udp_config.local_ip);
                    node->get_parameter(prefix + ".local_port", config.udp_config.local_port);
                    node->get_parameter(prefix + ".remote_ip", config.udp_config.remote_ip);
                    node->get_parameter(prefix + ".remote_port", config.udp_config.remote_port);
                    node->get_parameter(prefix + ".service", config.udp_config.service);
                    node->get_parameter(prefix + ".vsomeip_config", config.udp_config.vsomeip_config);
                } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
                    RCLCPP_WARN(node->get_logger(), "Radar %s missing UDP config: %s, skipping", config.radar_id.c_str(), e.what());
                    continue;
                }
            } else if (config.protocol == "can") {
                try {
                    node->get_parameter(prefix + ".device", config.can_config.device);
                    node->get_parameter(prefix + ".channel", config.can_config.channel);
                    node->get_parameter(prefix + ".arb_bitrate", config.can_config.arb_bitrate);
                    node->get_parameter(prefix + ".data_bitrate", config.can_config.data_bitrate);
                    node->get_parameter(prefix + ".dbc_file", config.can_config.dbc_file);

                    node->get_parameter(prefix + ".parse_rule.begin_message", config.can_config.begin_message);
                    node->get_parameter(prefix + ".parse_rule.end_message", config.can_config.end_message);
                    node->get_parameter(prefix + ".parse_rule.object_message", config.can_config.object_message);
                    node->get_parameter(prefix + ".parse_rule.id_signal", config.can_config.obj_id_signal);
                    node->get_parameter(prefix + ".parse_rule.x_signal", config.can_config.obj_x_signal);
                    node->get_parameter(prefix + ".parse_rule.y_signal", config.can_config.obj_y_signal);
                    node->get_parameter(prefix + ".parse_rule.exist_signal", config.can_config.obj_exist_signal);
                    node->get_parameter(prefix + ".parse_rule.valid_signal", config.can_config.obj_valid_signal);
                    node->get_parameter(prefix + ".parse_rule.obj_idx_pos", config.can_config.obj_idx_pos);

                } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
                    RCLCPP_WARN(node->get_logger(), "Radar %s missing CAN config: %s, skipping", config.radar_id.c_str(), e.what());
                    continue;
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Radar %s uses unknown protocol: %s, skipping", config.radar_id.c_str(), config.protocol.c_str());
                continue;
            }
            radar_configs.push_back(config);
        }

    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to load or parse YAML file: %s", e.what());
        throw; // Throw exception for main function to handle
    }

    return radar_configs;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("radar_muniu");

    // Declare and get config file path
    node->declare_parameter<std::string>("config_path",
        "./install/radar_muniu/share/radar_muniu/config/radar_muniu_config.yaml");
    std::string config_path;
    node->get_parameter("config_path", config_path);

    // Check if config file exists
    if (!rcutils_exists(config_path.c_str())) {
        RCLCPP_FATAL(node->get_logger(), "Config file not found: %s", config_path.c_str());
        rclcpp::shutdown();
        return -1;
    }

    std::vector<RadarConfig> radar_configs;

    try {
        // Call parameter parsing function
        radar_configs = loadRadarParameters(node.get());
    } catch (...) {
        rclcpp::shutdown();
        return -1;
    }

    // Create and start radar parsers
    std::vector<std::shared_ptr<radar_muniu::RadarParser>> radars;
    for (const auto& config : radar_configs) {
        if (config.protocol == "udp" || config.protocol == "can") {
            auto radar = std::make_shared<radar_muniu::RadarParser>(node.get(), config);
            if (!radar->start()) {
                RCLCPP_ERROR(node->get_logger(), "Radar %s start failed", config.radar_id.c_str());
                continue;
            }
            radars.push_back(radar);
        }
    }

    if (radars.empty()) {
        RCLCPP_FATAL(node->get_logger(), "No valid radar started");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "radar_muniu node started with %zu radars", radars.size());
    rclcpp::spin(node);

    // Stop radars and unsubscribe
    for (const auto& radar : radars) {
        radar->stop();
    }

    rclcpp::shutdown();
    return 0;
}