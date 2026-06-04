#pragma once
#include <vector>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace radar_muniu
{
    using DataCallback = std::function<void(uint32_t id, const std::vector<uint8_t>&)>;
    
    struct CANConfig
    {
        std::string device    = "";   // Device name (e.g., PCAN_USBBUS1 or can0)
        uint32_t arb_bitrate  = 0;    // Arbitration bitrate
        uint32_t data_bitrate = 0;    // Data bitrate
        uint8_t  channel      = 0;    // Channel number
        bool     is_canfd     = false;// Whether it is CAN FD
        std::string dbc_file  = "";   // DBC file path
        
        // Parsing rules
        std::vector<std::int64_t> begin_message;        // Start message IDs
        std::vector<std::int64_t> end_message;          // End message IDs
        std::vector<std::int64_t> object_message;       // Object message IDs
        std::string obj_id_signal;                      // Object ID signal name
        std::string obj_x_signal;                       // Object X coordinate signal name
        std::string obj_y_signal;                       // Object Y coordinate signal name
        std::string obj_exist_signal;                   // Object existence signal name
        std::string obj_valid_signal;                   // Object validity signal name
        uint8_t obj_idx_pos;                            // Object index position (offset)
    };

    class CANInterface
    {
    public:
        virtual ~CANInterface() = default;
        
        virtual bool init(rclcpp::Node* node, const CANConfig& config) = 0;

        virtual void start_receive(const DataCallback& callback) = 0;

        virtual bool send(const std::vector<uint8_t>& data) = 0;
        
        virtual void close() = 0;
        
        virtual bool is_running() const = 0;
    };

    std::unique_ptr<CANInterface> create_socketcan();
    std::unique_ptr<CANInterface> create_pcan();

}  // namespace radar_muniu