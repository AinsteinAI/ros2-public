#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/detection_list.hpp"
#include "radar_msgs/msg/object_list.hpp"
#include "radar_msgs/msg/dbc_message.hpp"
#include "radar_msgs/msg/version.hpp"

class RadarSubscriber : public rclcpp::Node
{
public:
	RadarSubscriber() : Node("radar_subscriber")
	{
		// Create QoS configuration matching sensor data
		auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
		// Try to set reliability policy to Best Effort (if used by the publisher)
		qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);


		std::string topic;

		// Subscribe to detection_list topic
		i79_detection_sub_ = this->create_subscription<radar_msgs::msg::DetectionList>(
			"/radar_muniu_i79/data/detection_list",
			qos,  // Use configured QoS
			std::bind(&RadarSubscriber::detection_callback, this, std::placeholders::_1)
		);

		// Subscribe to object_list topic
		i79_object_sub_ = this->create_subscription<radar_msgs::msg::ObjectList>(
			"/radar_muniu_i79/data/object_list",
			qos,  // Use configured QoS
			std::bind(&RadarSubscriber::object_callback, this, std::placeholders::_1)
		);
		// Subscribe to version_info topic
		i79_version_sub_ = this->create_subscription<radar_msgs::msg::Version>(
			"/radar_muniu_i79/data/version_info",
			qos,  // Use configured QoS
			std::bind(&RadarSubscriber::version_callback, this, std::placeholders::_1)
		);


		// Subscribe to dbc message topic
		const std::string topic_k77 = "/radar_muniu_k77/data/object_list";
		k77_dbc_sub_ = this->create_subscription<radar_msgs::msg::DbcMessage>(
			topic_k77,
			qos,  // Use configured QoS
			std::bind(&RadarSubscriber::dbcmsg_callback, this, std::placeholders::_1)
		);

		const std::string topic_t79 = "/radar_muniu_t79/data/object_list";
		t79_dbc_sub_ = this->create_subscription<radar_msgs::msg::DbcMessage>(
			topic_t79,
			qos,  // Use configured QoS
			std::bind(&RadarSubscriber::dbcmsg_callback, this, std::placeholders::_1)
		);

		

		RCLCPP_INFO(this->get_logger(), "Radar subscriber initialized");
	}

private:
	// Callback function for detection_list
	void detection_callback(const radar_msgs::msg::DetectionList::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "\n===============================");
		RCLCPP_INFO(this->get_logger(), "Received DetectionList: %d targets",
			msg->list_num_of_detections);

		if (!msg->list_detections.empty()) {
			auto& first_detection = msg->list_detections[0];
			RCLCPP_INFO(this->get_logger(),
				"First detection - range: %.2f, azimuth: %.2f elevation: %.2f",
				first_detection.range,
				first_detection.azimuth_angle,
				first_detection.elevation_angle);
		}
	}

	// Callback function for object_list
	void object_callback(const radar_msgs::msg::ObjectList::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "\n===============================");
		RCLCPP_INFO(this->get_logger(), "Received ObjectList: %d objects",
			msg->objectlist_numofobjects);

		if (!msg->objectlist_objects.empty()) {
			for (const auto& object : msg->objectlist_objects) {
				RCLCPP_INFO(this->get_logger(),
					"object - x: %6.2f, y: %6.2f, z: %6.2f",
					object.position_x,
					object.position_y,
					object.position_z);
			}
		}
	}

	// Callback function for can message
	void dbcmsg_callback(const radar_msgs::msg::DbcMessage::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "\n==============================================================");
		RCLCPP_INFO(this->get_logger(), "Received %s Message: (0x%02x) -> %ld signals",
			msg.get()->name.c_str(),
			msg.get()->id,
			msg->signals.size());

		if (!msg->signals.empty()) {
			for (const auto& signal : msg->signals) {
				RCLCPP_INFO(this->get_logger(), "\t: %s = %.3f %s",
					signal.name.c_str(),
					signal.value,
					signal.unit.c_str());
			}
		}
	}

	template <size_t N>
	std::string version_array_to_str(const std::array<uint8_t, N>& version_array) {
		std::stringstream ss;
		for (size_t i = 0; i < N; ++i) {
			ss << static_cast<int>(version_array[i]);  // Convert to decimal integer
			if (i != N - 1) {
				ss << ".";  // Separate version numbers with dots
			}
		}
		return ss.str();
	}

	void version_callback(const radar_msgs::msg::Version::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "\n===============================");

		RCLCPP_INFO(this->get_logger(), "Received Version:");
		RCLCPP_INFO(this->get_logger(), "  Protocol Version: %s", 
			version_array_to_str(msg.get()->protocol_version).c_str());
		RCLCPP_INFO(this->get_logger(), "  HW PCB Version:   %s", 
			version_array_to_str(msg.get()->hw_pcb_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  HW BOM Version:   %s", 
			version_array_to_str(msg.get()->hw_bom_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  CFG Version:      %s", 
			version_array_to_str(msg.get()->cfg_version).c_str());
		RCLCPP_INFO(this->get_logger(), "  SW PL Version:    %s", 
			version_array_to_str(msg.get()->sw_pl_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  SW PS Version:    %s", 
			version_array_to_str(msg.get()->sw_ps_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  ALG PT Version:   %s", 
			version_array_to_str(msg.get()->alg_pt_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  ALG TRK Version:  %s", 
			version_array_to_str(msg.get()->alg_trk_ver).c_str());
		RCLCPP_INFO(this->get_logger(), "  Calib Version:    %s", 
			version_array_to_str(msg.get()->calib_ver).c_str());

		std::stringstream sn_stream;
		for (size_t i = 0; i < msg.get()->sn_shell.size(); ++i) {
			uint8_t byte = msg.get()->sn_shell[i];
			if (std::isprint(static_cast<unsigned char>(byte))) {
				sn_stream << static_cast<char>(byte);
			} else {
				sn_stream << '*';
			}
			if ((i + 1) % 4 == 0 && i != msg.get()->sn_shell.size() - 1) {
				sn_stream << "";
			}
		}
		RCLCPP_INFO(this->get_logger(), "  Serial Number:    %s", sn_stream.str().c_str());
	}

	// Subscriber objects
	rclcpp::Subscription<radar_msgs::msg::DetectionList>::SharedPtr i79_detection_sub_;
	rclcpp::Subscription<radar_msgs::msg::ObjectList>::SharedPtr i79_object_sub_;
	rclcpp::Subscription<radar_msgs::msg::Version>::SharedPtr i79_version_sub_;
	rclcpp::Subscription<radar_msgs::msg::DbcMessage>::SharedPtr k77_dbc_sub_;
	rclcpp::Subscription<radar_msgs::msg::DbcMessage>::SharedPtr t79_dbc_sub_;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarSubscriber>());
	rclcpp::shutdown();
	return 0;
}