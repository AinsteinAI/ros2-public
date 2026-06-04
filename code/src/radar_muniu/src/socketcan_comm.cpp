// socket_can.cpp
#include "radar_muniu/socketcan_comm.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>
#ifndef _WIN32
	#include <fcntl.h>
	#include <sys/ioctl.h>
	#include <sys/socket.h>
	#include <unistd.h>
#endif

namespace radar_muniu
{
    SocketCANComm::SocketCANComm(rclcpp::Node* node, const CANConfig& config)
        : node_(node), config_(config), is_running_(false)
    {}

    SocketCANComm::~SocketCANComm()
    {
        close();
    }

    bool SocketCANComm::init()
    {
#ifndef _WIN32
        // Create CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create CAN socket: %s", strerror(errno));
            return false;
        }

        // Set CAN interface name
        std::string can_interface = "can" + std::to_string(config_.channel);
        strncpy(ifr_.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
        ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
        
        // Get interface index
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get interface index: %s", strerror(errno));
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Bind socket to CAN interface
        memset(&addr_, 0, sizeof(addr_));
        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to bind CAN socket: %s", strerror(errno));
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Set non-blocking mode
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags < 0) {
            RCLCPP_ERROR(node_->get_logger(), "fcntl F_GETFL failed: %s", strerror(errno));
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
        if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "fcntl F_SETFL failed: %s", strerror(errno));
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Configure CAN FD
        config_.is_canfd = config_.data_bitrate > 0;
        if (config_.is_canfd && !enable_canfd()) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Set baud rate
        if (!set_baudrate()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set CAN baudrate");
            ::close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "SocketCAN initialized successfully on %s", config_.device.c_str());
#endif
        return true;
    }

    void SocketCANComm::start_receive(const DataCallback& callback)
    {
#ifndef _WIN32
        if (is_running_) return;
        if (socket_fd_ < 0) return;

        data_callback_ = callback;
        is_running_ = true;
        recv_thread_ = std::thread(&SocketCANComm::receive_thread, this);
#endif
    }

    void SocketCANComm::receive_thread()
    {
#ifndef _WIN32
        while (is_running_) {
            if (config_.is_canfd) {
                struct canfd_frame frame;
                ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct canfd_frame));
                if (nbytes < 0) {
                    if (errno == EAGAIN) {
                        // No data available in non-blocking mode, sleep briefly
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                        continue;
                    } else if (errno != EINTR) {
                        RCLCPP_ERROR(node_->get_logger(), "CAN FD read error: %s", strerror(errno));
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    continue;
                }
                if (nbytes != sizeof(struct canfd_frame)) {
                    RCLCPP_ERROR(node_->get_logger(), "Incomplete CAN FD frame (read %zd bytes)", nbytes);
                    continue;
                }
                // Process received frame
                std::vector<uint8_t> data(frame.data, frame.data + frame.len);
                if (data_callback_) {
                    data_callback_(frame.can_id, data);
                }
            } else {
                struct can_frame frame;
                ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
                
                if (nbytes < 0) {
                    if (errno == EAGAIN) {
                        // No data available in non-blocking mode, sleep briefly
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                        continue;
                    } else if (errno != EINTR) {
                        RCLCPP_ERROR(node_->get_logger(), "CAN read error: %s", strerror(errno));
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    continue;
                }

                if (nbytes != sizeof(struct can_frame)) {
                    RCLCPP_ERROR(node_->get_logger(), "Incomplete CAN frame (read %zd bytes)", nbytes);
                    continue;
                }

                // Process received frame
                std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);
                if (data_callback_) {
                    data_callback_(frame.can_id, data);
                }
            }
        }
#endif
    }

    bool SocketCANComm::send(uint32_t id, const std::vector<uint8_t>& data)
    {
#ifndef _WIN32
        if (!is_running_ || socket_fd_ < 0) return false;

        int retries = 3;
        while (retries-- > 0) {
            if (config_.is_canfd) {
                struct canfd_frame frame;
                memset(&frame, 0, sizeof(frame));
                
                frame.can_id = id;
                if (id > 0x7FF) {
                    frame.can_id |= CAN_EFF_FLAG;
                }
                
                frame.len = data.size() > CANFD_MAX_DLEN ? CANFD_MAX_DLEN : data.size();
                memcpy(frame.data, data.data(), frame.len);

                ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct canfd_frame));
                if (nbytes == sizeof(struct canfd_frame)) {
                    return true;
                }
            } else {
                struct can_frame frame;
                memset(&frame, 0, sizeof(frame));
                
                frame.can_id = id;
                if (id > 0x7FF) {
                    frame.can_id |= CAN_EFF_FLAG;
                }
                
                frame.can_dlc = data.size() > CAN_MAX_DLEN ? CAN_MAX_DLEN : data.size();
                memcpy(frame.data, data.data(), frame.can_dlc);

                ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
                if (nbytes == sizeof(struct can_frame)) {
                    return true;
                }
            }
            
            RCLCPP_WARN(node_->get_logger(), "CAN send failed, retrying... (%d retries left)", retries);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        RCLCPP_ERROR(node_->get_logger(), "CAN send failed after retries: %s", strerror(errno));
#endif
        return false;
    }

    void SocketCANComm::close()
    {
#ifndef _WIN32
        if (is_running_) {
            is_running_ = false;
            if (recv_thread_.joinable()) {
                recv_thread_.join();
            }
        }

        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            RCLCPP_INFO(node_->get_logger(), "SocketCAN closed");
        }
#endif
    }

    bool SocketCANComm::set_baudrate()
    {
        // For Linux systems, the baud rate is usually set at the system level via the ip command
        // You can add logic to set the baud rate via ioctl as needed here
        if (config_.is_canfd) {
            return enable_canfd();
        }
        return true;
    }

    bool SocketCANComm::enable_canfd()
    {
#ifndef _WIN32
        int fd_option = 1;  // Enable CAN FD frames
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_option, sizeof(fd_option)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Enable CAN FD frame failed: %s", strerror(errno));
            return false;
        }
#endif
        return true;
    }
}  // namespace radar_muniu
