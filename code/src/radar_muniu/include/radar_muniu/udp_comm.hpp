#pragma once

#include <asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include "comm_interface.hpp"

namespace radar_muniu
{

	struct UDPConfig
	{
		std::string local_ip;
		uint16_t local_port;
		std::string remote_ip;
		uint16_t remote_port;

		std::vector<int64_t> service;
		std::string vsomeip_config;

		// Auto-reconnection configuration
		int max_reconnect_attempts = 5;      // Maximum number of reconnection attempts
		int reconnect_delay = 3;             // Reconnection delay (seconds)
		int receive_timeout = 10;            // Receive timeout (seconds)
		int heartbeat_interval = 5;          // Heartbeat interval (seconds)
		std::string heartbeat_data;          // Heartbeat data
	};

    struct UDPStats
    {
        size_t total_received;
        size_t total_sent;
        int reconnect_count;
        std::string last_error;
    };

    using ConnectionCallback = std::function<void(bool connected)>;

    class UDPComm : public CommInterface
    {
    public:
        using DataCallback = std::function<void(uint32_t id, const std::vector<uint8_t>&)>;

        UDPComm(rclcpp::Node* node, const UDPConfig& config);
        ~UDPComm();

        bool init();
        void start_receive(const DataCallback& callback);
        bool send(uint32_t id, const std::vector<uint8_t>& data);
        void close();
        UDPStats get_stats() const;
        bool is_connected() const;
        void set_connection_callback(const ConnectionCallback& callback);

    private:
        bool connect();
        bool send_raw(const std::vector<uint8_t>& data);
        void do_receive();
        void start_connection_monitor();
        void check_connection_health();
        void check_connection_status();
        void attempt_reconnect();
        void send_heartbeat();

        rclcpp::Node* node_;
        UDPConfig config_;
        asio::io_context io_ctx_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint remote_ep_;
        asio::steady_timer reconnect_timer_;
        asio::steady_timer heartbeat_timer_;

        std::array<uint8_t, 8192> recv_buf_;
        DataCallback data_callback_;
        std::thread io_thread_;
        std::mutex socket_mutex_;

        bool is_running_;
        int reconnect_attempts_;
        std::chrono::steady_clock::time_point last_receive_time_;
        UDPStats stats_;
        ConnectionCallback connection_callback_;
    };

}  // namespace radar_muniu