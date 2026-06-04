// peakcan_comm.cpp
#include "radar_muniu/peakcan_comm.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <map>
#include <thread>
#include <chrono>


namespace radar_muniu
{
  std::map<std::string, std::string> mapfdStr = {
    {"1M_10M",   "f_clock_mhz=80,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=1,data_tseg1=5,data_tseg2=2,data_sjw=1"},
    {"1M_5M",    "f_clock_mhz=80,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=2,data_tseg1=7,data_tseg2=2,data_sjw=1"},
    {"1M_2M",    "f_clock_mhz=80,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1"},
    {"500K_2M",  "f_clock_mhz=80,nom_brp=10,nom_tseg1=12,nom_tseg2=3,nom_sjw=1,data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1"},
    {"500K_5M",  "f_clock_mhz=80,nom_brp=10,nom_tseg1=12,nom_tseg2=3,nom_sjw=1,data_brp=2,data_tseg1=5,data_tseg2=2,data_sjw=1"},
    {"500K_10M", "f_clock_mhz=80,nom_brp=10,nom_tseg1=12,nom_tseg2=3,nom_sjw=1,data_brp=1,data_tseg1=5,data_tseg2=2,data_sjw=1"},
    };

    PeakCanComm::PeakCanComm(rclcpp::Node* node, const CANConfig& config)
        : node_(node), 
        config_(config), 
        handle_(PCAN_NONEBUS), 
        is_running_(false)
    {

    }

    PeakCanComm::~PeakCanComm() 
    { 
        close(); 
    }

    bool PeakCanComm::init()
    {
        TPCANBaudrate   Btr0Btr1 = PCAN_BAUD_500K;
        TPCANType       HwType = PCAN_TYPE_ISA;
        DWORD           IOPort = 0x2A0;
        WORD            Interrupt = 11;
        std::string     fdStr;

        int type = 0x00;
        int pos = config_.channel;
        if (type == 0) {
            if (pos != 0) {
                config_.channel = 0x50 + pos;
            }
            else
                config_.channel = 0x0;
        }
        else if (type == 1) {
            if (pos != 0) {
                config_.channel = 0x40 + pos;
            }
            HwType = PCAN_TYPE_ISA;
            IOPort = 0x100;
            Interrupt = 3;
        }

        int baudRate = config_.arb_bitrate;
        int nBaudRate = config_.data_bitrate;
        if (nBaudRate == 0) {
            if (baudRate == 250) {
                Btr0Btr1 = PCAN_BAUD_250K;
            }
            else if (baudRate == 500) {
                Btr0Btr1 = PCAN_BAUD_500K;
            }
            else if (baudRate == 1000) {
                Btr0Btr1 = PCAN_BAUD_1M;
            }
        }
        else {
            std::stringstream ss;
            std::string abr;
            if (baudRate % 1000 == 0) {
                ss.str(std::string());
                ss << baudRate / 1000 << "M";
                abr = ss.str();
            }
            else {
                ss.str(std::string());
                ss << baudRate << "K";
                abr = ss.str();
            }

            std::string dbr;
            if (nBaudRate % 1000 == 0) {
                ss.str(std::string());
                ss << nBaudRate / 1000 << "M";
                dbr = ss.str();
            }
            else {
                ss.str(std::string());
                ss << nBaudRate / 1000 << "K";
                dbr = ss.str();
             }

            std::string key = std::string(abr) + "_" + std::string(dbr);
            if (mapfdStr.find(key) != mapfdStr.end()) {
                fdStr = mapfdStr[key];
            }
        }

        TPCANStatus ret;
        if (fdStr.empty()) {
            ret = CAN_Initialize(config_.channel, Btr0Btr1, HwType, IOPort, Interrupt);
            config_.is_canfd = false;
        }
        else {
            std::cout << config_.channel << fdStr << std::endl;
            ret = CAN_InitializeFD(config_.channel, (TPCANBitrateFD)fdStr.c_str());
            config_.is_canfd = true;
        }

        handle_ = PCAN_USBBUS1;

        RCLCPP_INFO(node_->get_logger(), "CAN_Initialize  %d",  ret);

        return ret == PCAN_ERROR_OK;
    }

    void PeakCanComm::start_receive(const DataCallback& callback)
    {
        if (is_running_ || handle_ == PCAN_NONEBUS) return;
        data_callback_ = callback;
        is_running_ = true;
        recv_thread_ = std::thread(&PeakCanComm::receive_thread, this);
    }

    void PeakCanComm::receive_thread()
    {
        if (config_.is_canfd)
        {
            while (is_running_)
            {
                TPCANMsgFD msg = {};
                TPCANTimestampFD stamp = {};
                TPCANStatus ret = CAN_ReadFD(config_.channel, &msg, &stamp);
                if (ret == PCAN_ERROR_OK) {
                    if ((msg.MSGTYPE & PCAN_MESSAGE_ERRFRAME) || (msg.MSGTYPE & PCAN_MESSAGE_STATUS)) {
                        RCLCPP_ERROR(node_->get_logger(), "PCANBasicClass::readMsgFD: msg type %d", msg.MSGTYPE);
                        std::this_thread::sleep_for(std::chrono::microseconds(20));
                    }
                    else {
                        if (msg.DLC == 0) {
                            RCLCPP_ERROR(node_->get_logger(), "PCANBasicClass::readMsgFD: msg DLC %d", msg.DLC);
                        }
                        else {
                            static const std::vector<uint8_t>DlcToDataBytes = {
                                0,  1,  2,  3,  4,  5,  6,  7,
                                8, 12, 16, 20, 24, 32, 48, 64,
                            };

                            auto len = DlcToDataBytes.at(msg.DLC);
                            std::vector<uint8_t> data(msg.DATA, msg.DATA + len);
                            if (data_callback_)
                                data_callback_(msg.ID, data);
                        }
                    }
                }
                else if (ret == PCAN_ERROR_QRCVEMPTY) {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                } 
            } 
        }
        else
        {
            TPCANMsg frame;
            TPCANTimestamp timestamp;
            while (is_running_)
            {
                TPCANMsg msg = {};
                TPCANTimestamp stamp = {};;
                TPCANStatus ret = CAN_Read(config_.channel, &msg, &stamp);
                if (ret == PCAN_ERROR_OK && msg.LEN > 0) {
                    if ((msg.MSGTYPE & PCAN_MESSAGE_ERRFRAME) || (msg.MSGTYPE & PCAN_MESSAGE_STATUS)) {
                        RCLCPP_ERROR(node_->get_logger(), "PCANBasicClass::readMsg: msg type %d", msg.MSGTYPE);
                    }
                    else {
                        if (msg.LEN == 0) {
                            RCLCPP_ERROR(node_->get_logger(), "PCANBasicClass::readMsg: msg DLC %d", msg.LEN);
                        }
                        else {
                            std::vector<uint8_t> data(&msg.DATA[0], &msg.DATA[0] + (int)msg.LEN);
                            if (data_callback_) {
                                data_callback_(msg.ID, data);
                            }
                        }
                    }

                }
            }
        }
    }

    bool PeakCanComm::send(uint32_t id, const std::vector<uint8_t>& data)
    {
        if (!is_running_ || handle_ == PCAN_NONEBUS) return false;

        TPCANStatus ret = 0;

        int num = 3;
        if (!config_.is_canfd) {
            TPCANMsg tmp;
            tmp.ID = id;
            tmp.MSGTYPE = PCAN_MESSAGE_STANDARD;
            if (id > 0x7FF) {
                tmp.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
            }

            tmp.LEN = data.size() > 8 ? 8 : data.size();
            memcpy(tmp.DATA, data.data(), data.size());

            do {
                ret = CAN_Write(config_.channel, &tmp);
                if (ret == PCAN_ERROR_OK || --num <= 0) {
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } while (true);

            return ret == PCAN_ERROR_OK;
        }

        do {
            TPCANMsgFD tmpFD;
            tmpFD.ID = id;
            tmpFD.MSGTYPE = PCAN_MESSAGE_FD;
            if (id > 0x7FF) {
                tmpFD.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
            }

            const std::vector <uint8_t> DlcToDataBytes = {
                0,  1,  2,  3,  4,  5,  6,  7,
                8, 12, 16, 20, 24, 32, 48, 64,
            };
            tmpFD.DLC = data.size();
            for (int i = 0; i < DlcToDataBytes.size(); i++) {
                if (DlcToDataBytes[i] == tmpFD.DLC) {
                    tmpFD.DLC = i;
                    break;
                }
            }

            memcpy(tmpFD.DATA, data.data(), data.size());
            ret = CAN_WriteFD(config_.channel, &tmpFD);
            if (ret == PCAN_ERROR_OK || --num <= 0) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } while (true);

        return ret == PCAN_ERROR_OK;
    }

    void PeakCanComm::close()
    {
        if (is_running_)
        {
            is_running_ = false;
            if (recv_thread_.joinable())
                recv_thread_.join();
        }

        if (handle_ != PCAN_NONEBUS)
        {
            CAN_Uninitialize(handle_);
            handle_ = PCAN_NONEBUS;
            RCLCPP_INFO(node_->get_logger(), "PCAN closed");
        }
    }
}  // namespace radar_muniu