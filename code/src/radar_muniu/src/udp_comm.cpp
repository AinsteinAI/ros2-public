// udp_comm.cpp
#include "radar_muniu/udp_comm.hpp"
#include <fstream>
#include <chrono>

namespace radar_muniu
{

    UDPComm::UDPComm(rclcpp::Node* node, const UDPConfig& config)
        : node_(node), config_(config),
        socket_(io_ctx_),
        is_running_(false),
        reconnect_timer_(io_ctx_),
        heartbeat_timer_(io_ctx_),
        reconnect_attempts_(0),
        last_receive_time_(std::chrono::steady_clock::now())
    {
        // Initialize statistical information
        stats_.total_received = 0;
        stats_.total_sent = 0;
        stats_.reconnect_count = 0;
        stats_.last_error = "";
    }

    UDPComm::~UDPComm()
    {
        close();
    }

    bool UDPComm::init()
    {
        return connect();
    }

    bool UDPComm::connect()
    {
        try {
            std::lock_guard<std::mutex> lock(socket_mutex_);

            // Close the socket first if it's already open
            if (socket_.is_open()) {
                socket_.close();
            }

            // Validate IP addresses
            asio::ip::address local_addr, remote_addr;
            try {
                local_addr = asio::ip::make_address(config_.local_ip);
                remote_addr = asio::ip::make_address(config_.remote_ip);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid IP address: %s", e.what());
                return false;
            }

            // Create endpoints
            asio::ip::udp::endpoint local_endpoint(local_addr, config_.local_port);
            remote_ep_ = asio::ip::udp::endpoint(remote_addr, config_.remote_port);

            // Open and bind the socket
            socket_.open(asio::ip::udp::v4());
            socket_.set_option(asio::socket_base::reuse_address(true));
            socket_.bind(local_endpoint);

            // Reset statistical information
            reconnect_attempts_ = 0;
            last_receive_time_ = std::chrono::steady_clock::now();
            stats_.reconnect_count++;

            RCLCPP_INFO(node_->get_logger(), "UDP connected: %s:%d -> %s:%d (reconnect: %d)",
                config_.local_ip.c_str(), config_.local_port,
                config_.remote_ip.c_str(), config_.remote_port,
                stats_.reconnect_count);

            return true;

        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "UDP connect failed: %s", e.what());
            stats_.last_error = e.what();
            return false;
        }
    }

    void UDPComm::start_receive(const DataCallback& callback)
    {
        if (is_running_) return;

        data_callback_ = callback;
        is_running_ = true;

        // Start the IO context thread
        io_thread_ = std::thread([this]() {
            while (is_running_ && io_ctx_.run()) {
                // IO context is running, the loop will exit if stopped
            }
            RCLCPP_INFO(node_->get_logger(), "IO context thread exited");
            });

        // Start receiving data
        do_receive();

        // Start connection monitoring
        start_connection_monitor();

        RCLCPP_INFO(node_->get_logger(), "UDP receive started");
    }

    void UDPComm::do_receive()
    {
        if (!is_running_) return;

        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (!socket_.is_open()) {
            RCLCPP_WARN(node_->get_logger(), "Socket not open, skipping receive");
            return;
        }

        socket_.async_receive_from(
            asio::buffer(recv_buf_), remote_ep_,
            [this](std::error_code ec, std::size_t len) {
                if (!ec && len > 0) {
                    // Update the last receive time
                    last_receive_time_ = std::chrono::steady_clock::now();
                    stats_.total_received++;

                    // Call the data callback
                    if (data_callback_) {
                        data_callback_(0x00, std::vector<uint8_t>(recv_buf_.begin(), recv_buf_.begin() + len));
                    }

                    // Reset the reconnect attempt count
                    reconnect_attempts_ = 0;

                }
                else if (ec != asio::error::operation_aborted) {
                    RCLCPP_WARN(node_->get_logger(), "UDP receive error: %s", ec.message().c_str());
                    stats_.last_error = ec.message();

                    // Check if reconnection is needed
                    check_connection_status();
                }

                // Continue receiving the next packet
                if (is_running_) {
                    do_receive();
                }
            });
    }

    void UDPComm::start_connection_monitor()
    {
        // Heartbeat detection timer
        heartbeat_timer_.expires_after(std::chrono::seconds(config_.heartbeat_interval));
        heartbeat_timer_.async_wait([this](std::error_code ec) {
            if (!ec && is_running_) {
                check_connection_health();
                start_connection_monitor(); // Restart the timer
            }
            });
    }

    void UDPComm::check_connection_health()
    {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_receive = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_receive_time_).count();

        // If no data is received for longer than the timeout period, consider the connection abnormal
        if (time_since_last_receive > config_.receive_timeout) {
            RCLCPP_WARN(node_->get_logger(),
                "No data received for %ld seconds, connection may be lost",
                time_since_last_receive);
            check_connection_status();
        }

        // Send heartbeat packet (if heartbeat data is configured)
        if (!config_.heartbeat_data.empty()) {
            send_heartbeat();
        }
    }

    void UDPComm::send_heartbeat()
    {
        if (!is_running_) return;

        try {
            std::vector<uint8_t> heartbeat_data(config_.heartbeat_data.begin(),
                config_.heartbeat_data.end());
            send_raw(heartbeat_data);
            RCLCPP_DEBUG(node_->get_logger(), "Heartbeat sent");
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Heartbeat send failed: %s", e.what());
        }
    }

    void UDPComm::check_connection_status()
    {
        reconnect_attempts_++;

        RCLCPP_WARN(node_->get_logger(),
            "Connection check: attempt %d/%d",
            reconnect_attempts_, config_.max_reconnect_attempts);

        if (reconnect_attempts_ >= config_.max_reconnect_attempts) {
            RCLCPP_ERROR(node_->get_logger(), "Max reconnection attempts reached, attempting reconnect");
            attempt_reconnect();
        }
    }

    void UDPComm::attempt_reconnect()
    {
        RCLCPP_INFO(node_->get_logger(), "Attempting to reconnect...");

        // Stop current IO operations
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (socket_.is_open()) {
            socket_.cancel();
            socket_.close();
        }

        // Reconnect after delay
        reconnect_timer_.expires_after(std::chrono::seconds(config_.reconnect_delay));
        reconnect_timer_.async_wait([this](std::error_code ec) {
            if (!ec && is_running_) {
                if (connect()) {
                    RCLCPP_INFO(node_->get_logger(), "Reconnect successful");
                    
                    if (connection_callback_) {
                        connection_callback_(true);
                    }
                    // Restart receiving
                    do_receive();
                }
                else {
                    RCLCPP_ERROR(node_->get_logger(), "Reconnect failed, will retry");
                    // Retry recursively
                    attempt_reconnect();
                }
            }
            });
    }

    bool UDPComm::send(uint32_t id, const std::vector<uint8_t>& data)
    {
        return send_raw(data);
    }

    bool UDPComm::send_raw(const std::vector<uint8_t>& data)
    {
        if (!is_running_) {
            RCLCPP_WARN(node_->get_logger(), "Cannot send: UDP not running");
            return false;
        }

        try {
            std::lock_guard<std::mutex> lock(socket_mutex_);

            if (!socket_.is_open()) {
                RCLCPP_WARN(node_->get_logger(), "Cannot send: socket not open");
                return false;
            }

            size_t sent = socket_.send_to(asio::buffer(data), remote_ep_);
            stats_.total_sent++;

            RCLCPP_DEBUG(node_->get_logger(), "UDP sent %zu bytes to %s:%d",
                sent, config_.remote_ip.c_str(), config_.remote_port);

            return true;

        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "UDP send failed: %s", e.what());
            stats_.last_error = e.what();

            // Check connection status when sending fails
            check_connection_status();
            return false;
        }
    }

    void UDPComm::close()
    {
        if (is_running_) {
            is_running_ = false;

            // Cancel all timers
            reconnect_timer_.cancel();
            heartbeat_timer_.cancel();

            // Stop the socket
            std::lock_guard<std::mutex> lock(socket_mutex_);
            if (socket_.is_open()) {
                socket_.cancel();
                socket_.close();
            }

            // Stop the IO context
            io_ctx_.stop();
            if (io_thread_.joinable()) {
                io_thread_.join();
            }

            RCLCPP_INFO(node_->get_logger(), "UDP closed. Stats: received=%lu, sent=%lu, reconnects=%d",
                stats_.total_received, stats_.total_sent, stats_.reconnect_count);
        }
    }

    UDPStats UDPComm::get_stats() const
    {
        return stats_;
    }

    bool UDPComm::is_connected() const
    {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_receive = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_receive_time_).count();

        return socket_.is_open() &&
            time_since_last_receive < config_.receive_timeout;
    }

    void UDPComm::set_connection_callback(const ConnectionCallback& callback)
    {
        connection_callback_ = callback;
    }

}  // namespace radar_muniu