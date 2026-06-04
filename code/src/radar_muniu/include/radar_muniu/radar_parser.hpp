// radar_parser.hpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "radar_msgs/msg/detection_list.hpp"
#include "radar_msgs/msg/object_list.hpp"
#include "radar_msgs/msg/version.hpp"
#include "radar_msgs/msg/dbc_message.hpp"
#include "radar_msgs/msg/dbc_signal.hpp"
#include "dbcppp/CApi.h"
#include "dbcppp/Network.h"
#include "comm_interface.hpp"
#include "udp_comm.hpp"
#include "someip_client.hpp"
#include "peakcan_comm.hpp"
#include "socketcan_comm.hpp"


namespace radar_muniu
{
	// Radar configuration structure (integrates communication + installation parameters)
	struct RadarConfig
	{
		std::string radar_id;
		std::string frame_id;
		std::string parent_frame_id;
		std::string protocol;
		std::string direct;
		UDPConfig udp_config;
		CANConfig can_config;
		// Installation pose
		double x = 0, y = 0, z = 0;
		double roll = 0, pitch = 0, yaw = 0;
	};

	struct RadarTrack
	{
		uint8_t id = 0;
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float exst_prob = 0.0f;
		uint8_t valid = 1;
	};

	class RadarParser
	{
	public:
		RadarParser(rclcpp::Node* node, const RadarConfig& config);
		~RadarParser() = default;

		bool start();

		void stop();

	private:
		// Communication data callback (parsing entry point)
		void on_data_received_i79(uint32_t id, const std::vector<uint8_t>& data);

		// Communication data callback (parsing entry point)
		void on_data_received_can(uint32_t id, const std::vector<uint8_t>& data);

		// Parse protocol frame (distinguish between DetectionList/ObjectList)
		void parse_protocol_frame(const std::vector<uint8_t>& data, uint16_t service_id, uint64_t frame_ts);

		// Parse DetectionList
		void parse_detection_list(const std::vector<uint8_t>& data, uint64_t frame_ts);

		// Parse ObjectList
		void parse_object_list(const std::vector<uint8_t>& data, uint64_t frame_ts);

		// Parse Version
		void parse_version_info(const std::vector<uint8_t>& data, uint64_t frame_ts);

		// Publish TF transform (radar coordinate system → parent coordinate system)
		void publish_tf();

		void on_connection_status_changed(bool connected);

		sensor_msgs::msg::PointCloud2 create_point_cloud(const std::string& frame_id, size_t point_count, const rclcpp::Time& timestamp);

		rclcpp::Node* node_;
		RadarConfig config_;
		std::unique_ptr<CommInterface> comm_;
		// ROS publishers
		rclcpp::Publisher<radar_msgs::msg::DetectionList>::SharedPtr det_list_pub_;
		rclcpp::Publisher<radar_msgs::msg::ObjectList>::SharedPtr obj_list_pub_;
		rclcpp::Publisher<radar_msgs::msg::Version>::SharedPtr ver_info_pub_;

		rclcpp::Publisher<radar_msgs::msg::DbcMessage>::SharedPtr dbc_msg_pub_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		// TF publish timer
		rclcpp::TimerBase::SharedPtr tf_timer_;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_cloud_pub_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_track_pub_;

	private:
		uint32_t m_session_id = 0;
		std::vector<uint8_t> m_buffer;
		std::unique_ptr<dbcppp::INetwork> m_net;
		std::map<uint8_t, RadarTrack> m_frame;
		std::unordered_map<uint64_t, const dbcppp::IMessage*> m_messages;
	};

}  // namespace radar_muniu