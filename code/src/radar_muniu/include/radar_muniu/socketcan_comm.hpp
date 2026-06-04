// socket_can.hpp
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "comm_interface.hpp"
#include "can_interface.hpp"
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <functional>
#ifndef _WIN32
	#include <linux/can.h>
	#include <linux/can/raw.h>
	#include <net/if.h>
#endif

namespace radar_muniu
{

    class SocketCANComm : public CommInterface
    {
    public:
        SocketCANComm(rclcpp::Node* node, const CANConfig& config);
        ~SocketCANComm();

        bool init();
        void start_receive(const DataCallback& callback);
        bool send(uint32_t id, const std::vector<uint8_t>& data);
        void close();

    private:
        void receive_thread();
        bool set_baudrate();
        bool enable_canfd();

        rclcpp::Node* node_;
        CANConfig config_;
        int socket_fd_ = -1;
#ifndef _WIN32
        struct sockaddr_can addr_;
        struct ifreq ifr_;
#endif
        std::thread recv_thread_;
        std::atomic<bool> is_running_;
        DataCallback data_callback_ = nullptr;
    };
}  // namespace radar_muniu