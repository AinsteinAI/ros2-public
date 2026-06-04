#include "radar_muniu/someip_client.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <set>

namespace radar_muniu {

    VSomeIpClient::VSomeIpClient(rclcpp::Node* node, const UDPConfig& config)
        : node_(node), config_(config),
        is_initialized_(false),
        is_receiving_(false) {
    }

    VSomeIpClient::~VSomeIpClient() {
        close();
    }

    bool VSomeIpClient::init() {
        if (is_initialized_) {
            std::cout << "VSomeIpClient already initialized" << std::endl;
            return true;
        }

        // Create vsomeip application instance
        app_ptr_ = vsomeip::runtime::get()->create_application("radar-client");
        if (!app_ptr_) {
            std::cerr << "Failed to create vsomeip application" << std::endl;
            return false;
        }

        // Initialize the application
        if (!app_ptr_->init()) {
            std::cerr << "Failed to initialize vsomeip application" << std::endl;
            return false;
        }

        // Register state callback function
        app_ptr_->register_state_handler(
            std::bind(&VSomeIpClient::on_state, this, std::placeholders::_1)
        );

        register_availability_handlers();

        app_ptr_->register_message_handler(
            vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD,
            std::bind(&VSomeIpClient::on_message, this, std::placeholders::_1)
        );
        
        is_initialized_ = true;
        std::cout << "VSomeIpClient initialized successfully" << std::endl;
        return true;
    }

    void VSomeIpClient::config_path(const std::string& path) {    
        #ifdef _WIN32
            _putenv_s("VSOMEIP_CONFIGURATION", path.c_str());
            _putenv_s("VSOMEIP_CONFIGURATION_MODULE", "");
        #else
            setenv("VSOMEIP_CONFIGURATION", path.c_str(), 1);
            unsetenv("VSOMEIP_CONFIGURATION_MODULE");
        #endif    
    }

    void VSomeIpClient::start_receive(const DataCallback& callback) {
        if (!is_initialized_) {
            std::cerr << "VSomeIpClient not initialized, call init() first" << std::endl;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            data_callback_ = callback;
        }

        if (!is_receiving_) {
            std::cout << "Starting VSomeIpClient receive loop..." << std::endl;
            is_receiving_ = true;

            // Start sub-thread (after the thread starts, start_receive returns immediately without blocking)
            vsomeip_thread_ = std::thread([this]() {
                app_ptr_->start();  // Block until stop() is called
                std::cout << "vsomeip thread exited" << std::endl;
            });
        }
    }

    bool VSomeIpClient::send(uint32_t id, const std::vector<uint8_t>& data) {
        if (!is_initialized_ || !is_receiving_) {
            std::cerr << "VSomeIpClient not ready for sending" << std::endl;
            return false;
        }

        // Map interface ID to vsomeip service ID and method ID
        vsomeip::service_t service_id = static_cast<vsomeip::service_t>(id);
        vsomeip::method_t method_id = METHOD_ID_CONFIG;

        // Create request message
        auto message = vsomeip::runtime::get()->create_request();
        message->set_service(service_id);
        message->set_instance(INSTANCE_ID);
        message->set_method(method_id);

        // Set payload data
        auto payload = vsomeip::runtime::get()->create_payload();
        payload->set_data(data);
        message->set_payload(payload);

        // Send message
        app_ptr_->send(message);

        std::cout << "Sent data to service 0x" << std::hex << service_id
            << ", data size: " << std::dec << data.size() << " bytes" << std::endl;
        return true;
    }

    void VSomeIpClient::close() {
        if (is_receiving_ && app_ptr_) {
            std::cout << "Closing VSomeIpClient..." << std::endl;
            app_ptr_->stop();
            is_receiving_ = false;

            if (vsomeip_thread_.joinable()) {
                vsomeip_thread_.join();
            }
        }

        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            data_callback_ = nullptr;
        }

        is_initialized_ = false;
        std::cout << "VSomeIpClient closed" << std::endl;
    }

    void VSomeIpClient::register_availability_handlers() {
        app_ptr_->register_availability_handler(
            vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE,
            std::bind(&VSomeIpClient::on_availability, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            1);
    }

    void VSomeIpClient::on_state(vsomeip::state_type_e state) {
        if (state == vsomeip::state_type_e::ST_REGISTERED) {
            std::cout << "Client registered, requesting services..." << std::endl;
            request_all_services();
        }
        else if (state == vsomeip::state_type_e::ST_DEREGISTERED) {
            std::cout << "Client deregistered, trying to reconnect..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            if (app_ptr_) {
                app_ptr_->init();
                app_ptr_->start();
            }
        }
    }

    void VSomeIpClient::request_all_services(int retry_count) {
        if (retry_count >= 5) {
            std::cout << "Service request retries exhausted" << std::endl;
            return;
        }

        app_ptr_->request_service(vsomeip::ANY_SERVICE, INSTANCE_ID);
        std::cout << "Requested service: 0x" << std::hex << vsomeip::ANY_SERVICE
            << " (retry " << retry_count << ")" << std::dec << std::endl;
    }

    void VSomeIpClient::on_availability(vsomeip::service_t service, vsomeip::instance_t instance, bool is_available) {
        std::cout << "Service [" << std::hex << service << "." << instance << "] "
            << (is_available ? "AVAILABLE" : "UNAVAILABLE") << std::dec << std::endl;

        if (is_available && std::binary_search(config_.service.begin(), config_.service.end(), service)) {
            subscribe_event(service, instance);
        }
    }

    void VSomeIpClient::subscribe_event(vsomeip::service_t service, vsomeip::instance_t instance) {
        auto it = EVENT_MAP.find(service);
        if (it == EVENT_MAP.end()) {
            return;
        }

        auto event_id = EVENT_MAP.at(service);

        std::set<vsomeip::eventgroup_t> event_groups;
        event_groups.insert(EVENTGROUP_ID);

        app_ptr_->request_event(
            service,
            instance,
            event_id,
            event_groups,
            vsomeip::event_type_e::ET_FIELD,
            vsomeip::reliability_type_e::RT_UNRELIABLE
        );

        app_ptr_->subscribe(service, instance, EVENTGROUP_ID, 0x01);
        std::cout << "Subscribed to event group 0x" << std::hex << EVENTGROUP_ID
            << " of service 0x" << service << std::dec << std::endl;
    }

    void VSomeIpClient::on_message(const std::shared_ptr<vsomeip::message>& message) {
        if (!message) return;

        auto message_type = message->get_message_type();
        if (message_type == vsomeip::message_type_e::MT_NOTIFICATION) {
            handle_notification_message(message);
        }
        else if (message_type == vsomeip::message_type_e::MT_RESPONSE) {
            handle_response_message(message);
        }
    }

    void VSomeIpClient::handle_notification_message(const std::shared_ptr<vsomeip::message>& message) {
        auto service_id = message->get_service();
        auto payload = message->get_payload();

        if (!payload) return;

        // Convert vsomeip payload to interface data format
        std::vector<uint8_t> data(payload->get_data(), payload->get_data() + payload->get_length());

        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (data_callback_) {
            data_callback_(static_cast<uint32_t>(service_id), data);
        }

        std::cout << "Received event from service 0x" << std::hex << service_id
            << ", data length: " << std::dec << payload->get_length() << std::endl;
    }

    void VSomeIpClient::handle_response_message(const std::shared_ptr<vsomeip::message>& message) {
        auto service_id = message->get_service();
        auto payload = message->get_payload();

        if (!payload) return;

        std::cout << "Received response from service 0x" << std::hex << service_id
            << ", data length: " << std::dec << payload->get_length() << std::endl;
    }

} // namespace radar_muniu