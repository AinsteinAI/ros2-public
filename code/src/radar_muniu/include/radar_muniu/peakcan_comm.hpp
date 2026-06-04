// peakcan_comm.hpp 修改部分
#pragma once
#include <thread>
#include <atomic>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif
#include "rclcpp/rclcpp.hpp"
#include "comm_interface.hpp"
#include "can_interface.hpp"
#include "../third_party/PCAN-Basic/Include/PCANBasic.h"

namespace radar_muniu
{
    class PeakCanComm : public CommInterface
    {
    public:
        PeakCanComm(rclcpp::Node* node, const CANConfig& config);
        ~PeakCanComm() override;

        virtual bool init() override;
        virtual void start_receive(const DataCallback& callback) override;
        virtual bool send(uint32_t id, const std::vector<uint8_t>& data) override;
        virtual void close() override;

    private:
        void receive_thread();
        
        rclcpp::Node* node_;
        CANConfig config_;
        TPCANHandle handle_;
        std::thread recv_thread_;
        std::atomic<bool> is_running_;
        DataCallback data_callback_ = nullptr;
    };
}  // namespace radar_muniu