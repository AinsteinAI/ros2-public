#ifndef RADAR_MUNIU_VSOMEIP_CLIENT_HPP
#define RADAR_MUNIU_VSOMEIP_CLIENT_HPP

#include "radar_muniu/comm_interface.hpp"
#include "radar_muniu/udp_comm.hpp"
#include <vsomeip/vsomeip.hpp>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>

namespace radar_muniu {
	// Service configuration
	static constexpr vsomeip::service_t SERVICE_DETECT_ID  = 0x200c;
	static constexpr vsomeip::service_t SERVICE_OBJECT_ID  = 0x300c;
	static constexpr vsomeip::service_t SERVICE_VEHIVLE_ID = 0x400c;
	static constexpr vsomeip::service_t SERVICE_MONITOR_ID = 0x500c;
	static constexpr vsomeip::service_t SERVICE_RTK_ID     = 0x600c;
	static constexpr vsomeip::service_t SERVICE_VERSION_ID = 0x700c;
	static constexpr vsomeip::instance_t INSTANCE_ID = 0x200c;
	static constexpr vsomeip::eventgroup_t EVENTGROUP_ID = 0x1101;


	class VSomeIpClient : public CommInterface {
	public:
		VSomeIpClient(rclcpp::Node* node, const UDPConfig& config);
		virtual ~VSomeIpClient();

		bool init() override;
		void start_receive(const DataCallback& callback) override;
		bool send(uint32_t id, const std::vector<uint8_t>& data) override;
		void close() override;
		void config_path(const std::string &path);

	private:
		void register_availability_handlers();
		void on_state(vsomeip::state_type_e state);
		void request_all_services(int retry_count = 0);
		void on_availability(vsomeip::service_t service, vsomeip::instance_t instance, bool is_available);
		void subscribe_event(vsomeip::service_t service, vsomeip::instance_t instance);
		void on_message(const std::shared_ptr<vsomeip::message>& message);

		void handle_notification_message(const std::shared_ptr<vsomeip::message>& message);
		void handle_response_message(const std::shared_ptr<vsomeip::message>& message); 

	private:
		std::shared_ptr<vsomeip::application> app_ptr_;
		std::thread vsomeip_thread_;

		DataCallback data_callback_;

		std::atomic<bool> is_initialized_;
		std::atomic<bool> is_receiving_; 
		std::mutex callback_mutex_; 


		rclcpp::Node* node_; 
		UDPConfig config_;


		// Method ID mapping (for sending requests)
		static constexpr vsomeip::method_t METHOD_ID_CONFIG = 0x0001;

		// Service name mapping (for logging/debugging)
		const std::map<vsomeip::service_t, std::string> SERVICE_MAP = {
			{SERVICE_DETECT_ID, "detection_list"},
			{SERVICE_OBJECT_ID, "object_list"},
			{SERVICE_VEHIVLE_ID, "VehicleInfo"},
			{SERVICE_MONITOR_ID, "Monitor"},
			{SERVICE_RTK_ID, "RtkInfor"},
			{SERVICE_VERSION_ID, "version_info"}
		};

		// Event ID mapping (for event subscription)
		const std::map<vsomeip::service_t, vsomeip::event_t> EVENT_MAP =
		{
			{SERVICE_DETECT_ID, 0x8001},
			{SERVICE_OBJECT_ID, 0x8002},
			{SERVICE_VEHIVLE_ID, 0x8003},
			{SERVICE_MONITOR_ID, 0x8004},
			{SERVICE_RTK_ID, 0x8005},
			{SERVICE_VERSION_ID, 0x8006}
		};
	};

} // namespace radar_muniu

#endif // RADAR_MUNIU_VSOMEIP_CLIENT_HPP