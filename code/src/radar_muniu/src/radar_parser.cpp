// radar_parser.cpp
#include "radar_muniu/radar_parser.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cstring>
#include "radar_muniu/radar_model.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono> 
#include <fstream> 
using namespace std::chrono_literals;

namespace radar_muniu
{
	RadarParser::RadarParser(rclcpp::Node* node, const RadarConfig& config)
		: node_(node), config_(config)
	{

		// Initialize publishers
		if (config.protocol == "udp") {
			for (auto service_id : config.udp_config.service) {
				if (service_id == SERVICE_DETECT_ID) {
					det_list_pub_ = node_->create_publisher<radar_msgs::msg::DetectionList>(
						"/" + config.radar_id + "/data/detection_list", rclcpp::SensorDataQoS());

					radar_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
						"/" + config.radar_id + "/vis/detection_list", 10
					);
				}
				else if (service_id == SERVICE_OBJECT_ID) {
					obj_list_pub_ = node_->create_publisher<radar_msgs::msg::ObjectList>(
						"/" + config.radar_id + "/data/object_list", rclcpp::SensorDataQoS());

					radar_track_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
						"/" + config.radar_id + "/vis/object_list",
						10
					);
				}
				else if (service_id == SERVICE_VERSION_ID) {
					ver_info_pub_ = node_->create_publisher<radar_msgs::msg::Version>(
						"/" + config.radar_id + "/data/version_info", rclcpp::SensorDataQoS());
				}
			}
		}
		else if (config.protocol == "can") {
			dbc_msg_pub_ = node_->create_publisher<radar_msgs::msg::DbcMessage>(
				"/" + config.radar_id + "/data/object_list", rclcpp::SensorDataQoS());
			radar_track_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
				"/" + config.radar_id + "/vis/object_list",
				10
			);
		}


		// Initialize TF broadcaster
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);


		// Dynamically create communication instance
		std::string package_share_dir = ament_index_cpp::get_package_share_directory("radar_muniu");
		if (config.protocol == "udp") {
			comm_ = std::make_unique<VSomeIpClient>(node, config.udp_config);

			std::string json_path = package_share_dir + "/config/" + config_.udp_config.vsomeip_config;
			dynamic_cast<VSomeIpClient*>(comm_.get())->config_path(json_path);
		}
		else if (config.protocol == "can") {
			if (config.can_config.device == "pcan")
				comm_ = std::make_unique<PeakCanComm>(node, config.can_config);
			else if (config.can_config.device == "socketcan")
				comm_ = std::make_unique<SocketCANComm>(node, config.can_config);

			// Load DBC file
			std::string dbc_path = package_share_dir + "/config/" + config.can_config.dbc_file;
			std::ifstream idbc(dbc_path);
			if (!idbc.is_open()) {
				RCLCPP_FATAL(node_->get_logger(), "Cannot open DBC file:%s", dbc_path.c_str());
				return;
			}

			m_net = dbcppp::INetwork::LoadDBCFromIs(idbc);
			if (!m_net) {
				RCLCPP_ERROR(node_->get_logger(), "load dbc failed");
			}
			else {
				for (const dbcppp::IMessage& msg : m_net->Messages()) {
					m_messages.insert(std::make_pair(msg.Id(), &msg));
				}
			}

		}
		else {
			RCLCPP_FATAL(node_->get_logger(), "Unsupported protocol: %s", config.protocol.c_str());
		}

		// Start TF publishing timer (100ms cycle)
		tf_timer_ = node_->create_wall_timer(
			100ms,
			std::bind(&RadarParser::publish_tf, this)
		);
	}

	bool RadarParser::start()
	{
		if (!comm_) return false;


		// Initialize communication
		if (!comm_->init()) return false;


		if (config_.protocol == "udp") {
			comm_->start_receive(std::bind(&RadarParser::on_data_received_i79, this, std::placeholders::_1, std::placeholders::_2));
		}
		else {
			comm_->start_receive(std::bind(&RadarParser::on_data_received_can, this, std::placeholders::_1, std::placeholders::_2));
		}


		RCLCPP_INFO(node_->get_logger(), "Radar [%s] started", config_.radar_id.c_str());
		return true;
	}

	void RadarParser::stop()
	{
		if (config_.protocol == "udp") {
			comm_->close();
		}
		else if (config_.protocol == "can") {
			comm_->close();
		}
		else {
			RCLCPP_FATAL(node_->get_logger(), "Unsupported protocol: %s", config_.protocol.c_str());
		}
	}

	void RadarParser::on_data_received_can(uint32_t id, const std::vector<uint8_t>& data)
	{
		if (config_.protocol != "can") {
			return;
		}

		auto iter = m_messages.find(id);
		if (iter == m_messages.end()) {
			return;
		}

		auto dbc_msg_msg = radar_msgs::msg::DbcMessage();
		dbc_msg_msg.header.stamp = node_->get_clock()->now();
		dbc_msg_msg.header.frame_id = config_.frame_id;
		dbc_msg_msg.id = id;

		auto beg_msg = config_.can_config.begin_message;
		auto end_msg = config_.can_config.end_message;
		auto obj_msg = config_.can_config.object_message;

		const dbcppp::IMessage* msg = iter->second;
		dbc_msg_msg.name = msg->Name();


		if (std::find(obj_msg.begin(), obj_msg.end(), id) != obj_msg.end()) {
			for (const dbcppp::ISignal& sig : msg->Signals()) {
				const dbcppp::ISignal* mux_sig = msg->MuxSignal();
				bool signal_valid = sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue
					|| (mux_sig && mux_sig->Decode(data.data()) == sig.MultiplexerSwitchValue());
				if (!signal_valid) {
					continue;
				}

				//std::cout << "\t" << sig.Name() << "=" << sig.RawToPhys(sig.Decode(data.data())) << sig.Unit() << "\n";
				auto dbc_signal = radar_msgs::msg::DbcSignal();
				dbc_signal.name = sig.Name();
				dbc_signal.value = sig.RawToPhys(sig.Decode(data.data()));
				dbc_signal.unit = sig.Unit();
				dbc_msg_msg.signals.push_back(dbc_signal);

				std::int64_t base_id = -1;
				auto str_seq = RadarModel::getNumberFromString(dbc_signal.name, config_.can_config.obj_idx_pos);
				if (str_seq.size() > 0) {
					base_id = std::stoi(str_seq);
				}
				if (base_id == -1) {
					continue;
				}

				RadarTrack& track = m_frame[base_id];
				if (RadarModel::check_signal(dbc_signal.name, config_.can_config.obj_id_signal)) {
					track.id = dbc_signal.value;
				}
				else if (RadarModel::check_signal(dbc_signal.name, config_.can_config.obj_exist_signal)) {
					track.exst_prob = dbc_signal.value;
				}
				else if (RadarModel::check_signal(dbc_signal.name, config_.can_config.obj_valid_signal)) {
					track.valid = static_cast<uint8_t>(dbc_signal.value);
				}
				else if (RadarModel::check_signal(dbc_signal.name, config_.can_config.obj_x_signal)) {
					track.x = dbc_signal.value;
				}
				else if (RadarModel::check_signal(dbc_signal.name, config_.can_config.obj_y_signal)) {
					track.y = dbc_signal.value;
				}
			}

			dbc_msg_pub_->publish(dbc_msg_msg);
			RCLCPP_INFO(node_->get_logger(), "Published %s Dbc Message ID %02x ", config_.radar_id.c_str(), id);
		}

		// First frame message
		if (std::find(beg_msg.begin(), beg_msg.end(), id) != beg_msg.end()) {
			return;
		}
		// Last frame message
		else if (std::find(end_msg.begin(), end_msg.end(), id) != end_msg.end()) {
			// Remove invalid targets
			for (auto it = m_frame.begin(); it != m_frame.end(); ) {
				if (it->second.valid == 0 || it->second.id == 0xFF) {
					it = m_frame.erase(it);
				}
				else {
					++it;
				}
			}
		}
		// Target frame message
		else if (std::find(obj_msg.begin(), obj_msg.end(), id) != obj_msg.end()) {
			return;
		}
		else {
			return;
		}

		auto cloud_msg = create_point_cloud(config_.frame_id, m_frame.size(), node_->get_clock()->now());
		size_t i = 0;
		for (auto const& pair : m_frame) {
			size_t data_offset = i++ * cloud_msg.point_step;

			float x = pair.second.x;
			float y = pair.second.y;
			float z = pair.second.z;
			// 4. Intensity value
			float intensity = pair.second.exst_prob;

			// Copy data to point cloud
			memcpy(&cloud_msg.data[data_offset + 0], &x, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 4], &y, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 8], &z, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 12], &intensity, sizeof(float));
		}

		// Publish tracked targets
		radar_track_pub_->publish(cloud_msg);
		RCLCPP_INFO(node_->get_logger(), "Published %s object with %zu points", config_.radar_id.c_str(),
			m_frame.size());

		m_frame.clear();
	}

	void RadarParser::on_data_received_i79(uint32_t id, const std::vector<uint8_t>& data)
	{
		parse_protocol_frame(data, id, 0);
	}

	void RadarParser::on_connection_status_changed(bool connected)
	{
		if (connected) {

		}
		else {
			RCLCPP_WARN(node_->get_logger(), "Connection lost, subscription will be resent when reconnected");

		}
	}

	void RadarParser::parse_protocol_frame(const std::vector<uint8_t>& data, uint16_t service_id, uint64_t frame_ts)
	{
		if (data.empty()) return;

		switch (service_id) {
		case SERVICE_DETECT_ID:
			parse_detection_list(data, frame_ts);
			break;
		case SERVICE_OBJECT_ID:
			parse_object_list(data, frame_ts);
			break;
		case SERVICE_VERSION_ID:
			parse_version_info(data, frame_ts);
		default:
			RCLCPP_WARN(node_->get_logger(), "Unknown service id: 0x%02X", service_id);
			break;
		}
	}

	void RadarParser::parse_detection_list(const std::vector<uint8_t>& data, uint64_t frame_ts)
	{
		constexpr size_t LIST_SIZE_LEN = 82071;
		if (data.size() != LIST_SIZE_LEN) {
			RCLCPP_WARN(node_->get_logger(), "DetectionList len mismatch: %zu vs %zu", data.size(), LIST_SIZE_LEN);
			return;
		}

		// Construct DetectionList message
		auto det_list_msg = radar_msgs::msg::DetectionList();
		det_list_msg.header.stamp = node_->get_clock()->now();
		det_list_msg.header.frame_id = config_.frame_id;

		uint8_t* p = (uint8_t*)data.data();
		p += RadarModel::convert(det_list_msg.crc, p);
		p += RadarModel::convert(det_list_msg.length, p);
		p += RadarModel::convert(det_list_msg.sqc, p);
		p += RadarModel::convert(det_list_msg.data_id, p);
		p += RadarModel::convert(det_list_msg.version_code, p);
		p += RadarModel::convert(det_list_msg.frame_count, p);
		p += RadarModel::convert(det_list_msg.frame_period_time, p);
		p += RadarModel::convert(det_list_msg.timestamp_nanoseconds, p);
		p += RadarModel::convert(det_list_msg.timestamp_seconds, p);
		p += RadarModel::convert(det_list_msg.timestamp_sync_status, p);
		p += RadarModel::convert(det_list_msg.rsv1, p);
		p += RadarModel::convert(det_list_msg.rsv2, p);
		p += RadarModel::convert(det_list_msg.coordinate_system, p);
		p += RadarModel::convert(det_list_msg.measurement_latency, p);
		p += RadarModel::convert(det_list_msg.origin_invalid_flags, p);
		p += RadarModel::convert(det_list_msg.origin_xpos, p);
		p += RadarModel::convert(det_list_msg.origin_xstd, p);
		p += RadarModel::convert(det_list_msg.origin_ypos, p);
		p += RadarModel::convert(det_list_msg.origin_ystd, p);
		p += RadarModel::convert(det_list_msg.origin_zpos, p);
		p += RadarModel::convert(det_list_msg.origin_zstd, p);
		p += RadarModel::convert(det_list_msg.origin_roll, p);
		p += RadarModel::convert(det_list_msg.origin_rollstd, p);
		p += RadarModel::convert(det_list_msg.origin_pitch, p);
		p += RadarModel::convert(det_list_msg.origin_pitchstd, p);
		p += RadarModel::convert(det_list_msg.origin_yaw, p);
		p += RadarModel::convert(det_list_msg.origin_yawstd, p);
		p += RadarModel::convert(det_list_msg.list_invalid_flags, p);
		p += RadarModel::convert(det_list_msg.aln_azimuth_correction, p);
		p += RadarModel::convert(det_list_msg.aln_elevation_correction, p);
		p += RadarModel::convert(det_list_msg.rsv_weather, p);
		p += RadarModel::convert(det_list_msg.rsv_road_status, p);
		p += RadarModel::convert(det_list_msg.rsv_aucc_frame_count, p);
		p += RadarModel::convert(det_list_msg.rsv_profile_id, p);
		p += RadarModel::convert(det_list_msg.rsv_adc_anti_num, p);
		p += RadarModel::convert(det_list_msg.rsv_adc_real_higer_thod_cnt, p);
		p += RadarModel::convert(det_list_msg.rsv_adc_imag_higer_thod_cnt, p);
		p += RadarModel::convert(det_list_msg.u_rsv_bfm_coeff_error_flag, p);
		p += RadarModel::convert(det_list_msg.rsv_ci_noise_median, p);
		p += RadarModel::convert(det_list_msg.rsv_nci_noise_median, p);
		p += RadarModel::convert(det_list_msg.rsv3, p);
		p += RadarModel::convert(det_list_msg.list_len_of_detections, p);
		p += RadarModel::convert(det_list_msg.list_num_of_detections, p);

		RCLCPP_INFO(node_->get_logger(), "detect frame_id %d", det_list_msg.frame_count);

		// Parse each Detection
		constexpr size_t DETECTION_LEN = 40;
		for (uint32_t i = 0; i < det_list_msg.list_num_of_detections; ++i) {

			radar_msgs::msg::Detection det;
			p += RadarModel::convert(det.measurement_id, p);
			p += RadarModel::convert(det.azimuth_angle, p);
			p += RadarModel::convert(det.azimuth_angle_std, p);
			p += RadarModel::convert(det.invalid_flags, p);
			p += RadarModel::convert(det.elevation_angle, p);
			p += RadarModel::convert(det.elevation_angle_std, p);
			p += RadarModel::convert(det.range, p);
			p += RadarModel::convert(det.range_std, p);
			p += RadarModel::convert(det.range_rate, p);
			p += RadarModel::convert(det.range_rate_std, p);
			p += RadarModel::convert(det.rcs, p);
			p += RadarModel::convert(det.snr, p);
			p += RadarModel::convert(det.positive_predictive_value, p);
			p += RadarModel::convert(det.classification, p);
			p += RadarModel::convert(det.ambiguity_flag, p);

			// Field range limitation
			//det.distance = std::clamp(det.distance, 0.0f, 500.0f);
			//det.angle = std::clamp(det.angle, -3.1416f, 3.1416f);
			//det.elv = std::clamp(det.elv, -3.1416f, 3.1416f);
			//det.confidence = std::clamp(det.confidence, 0.0f, 100.0f);

			det_list_msg.list_detections.push_back(det);
		}

		// Publish message
		det_list_pub_->publish(det_list_msg);
		RCLCPP_INFO(node_->get_logger(), "[%s] Published DetectionList: %d detection",
			config_.radar_id.c_str(), det_list_msg.list_num_of_detections);

	    // Populate point cloud data
		auto cloud_msg = create_point_cloud(config_.frame_id, det_list_msg.list_num_of_detections, node_->get_clock()->now());
		for (size_t i = 0; i < det_list_msg.list_detections.size(); ++i) {
			const auto& det = det_list_msg.list_detections[i];
			size_t data_offset = i * cloud_msg.point_step;

			// Spherical coordinates to Cartesian coordinates
			float azimuth = det.azimuth_angle;    // Azimuth angle (radians)
			float elevation = det.elevation_angle; // Elevation angle (radians)
			float range = det.range;

			float cos_elv = cos(elevation);
			float x = range * cos_elv * cos(azimuth);
			float y = range * cos_elv * sin(azimuth);
			float z = range * sin(elevation);

			// Intensity value
			float intensity = det.positive_predictive_value * 255.0f;

			// Copy data to point cloud
			memcpy(&cloud_msg.data[data_offset + 0], &x, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 4], &y, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 8], &z, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 12], &intensity, sizeof(float));
		}

		// Publish point cloud
		radar_cloud_pub_->publish(cloud_msg);
		RCLCPP_INFO(node_->get_logger(), "Published detection_list PointCloud2 with %zu points",
			det_list_msg.list_detections.size());
	}


	void RadarParser::parse_object_list(const std::vector<uint8_t>& data, uint64_t frame_ts)
	{
		constexpr size_t LIST_SIZE_LEN = 6626;
		if (data.size() != LIST_SIZE_LEN) {
			RCLCPP_WARN(node_->get_logger(), "ObjectList len mismatch: %zu vs %zu", data.size(), LIST_SIZE_LEN);
			return;
		}

		// Construct ObjectList message
		auto obj_list_msg = radar_msgs::msg::ObjectList();
		obj_list_msg.header.stamp = node_->get_clock()->now();
		obj_list_msg.header.frame_id = config_.frame_id;

		uint8_t* p = (uint8_t*)data.data();
		p += RadarModel::convert(obj_list_msg.crc, p);
		p += RadarModel::convert(obj_list_msg.length, p);
		p += RadarModel::convert(obj_list_msg.sqc, p);
		p += RadarModel::convert(obj_list_msg.dataid, p);
		p += RadarModel::convert(obj_list_msg.timestamp_nanoseconds, p);
		p += RadarModel::convert(obj_list_msg.timestamp_seconds, p);
		p += RadarModel::convert(obj_list_msg.timestamp_syncstatus, p);
		p += RadarModel::convert(obj_list_msg.rsv1, p);
		p += RadarModel::convert(obj_list_msg.rsv2, p);
		p += RadarModel::convert(obj_list_msg.frame_count, p);
		p += RadarModel::convert(obj_list_msg.coordinate_system, p);
		p += RadarModel::convert(obj_list_msg.measurement_latency, p);
		p += RadarModel::convert(obj_list_msg.rsv3, p);
		p += RadarModel::convert(obj_list_msg.objectlist_numofobjects, p);

		RCLCPP_INFO(node_->get_logger(), "object frame_id %d", obj_list_msg.frame_count);

		// Parse each Object
		constexpr size_t OBJECT_LEN = 131;
		for (uint16_t i = 0; i < obj_list_msg.objectlist_numofobjects; ++i) {
			radar_msgs::msg::Object obj;

			p += RadarModel::convert(obj.object_id, p);
			p += RadarModel::convert(obj.object_age, p);
			p += RadarModel::convert(obj.object_status_measurement, p);
			p += RadarModel::convert(obj.object_status_movement, p);
			p += RadarModel::convert(obj.position_reference, p);
			p += RadarModel::convert(obj.position_x, p);
			p += RadarModel::convert(obj.position_x_std, p);
			p += RadarModel::convert(obj.position_y, p);
			p += RadarModel::convert(obj.position_y_std, p);
			p += RadarModel::convert(obj.position_z, p);
			p += RadarModel::convert(obj.position_z_std, p);
			p += RadarModel::convert(obj.position_orientation, p);
			p += RadarModel::convert(obj.position_orientation_std, p);
			p += RadarModel::convert(obj.dynamics_vel_x, p);
			p += RadarModel::convert(obj.dynamics_vel_x_std, p);
			p += RadarModel::convert(obj.dynamics_vel_y, p);
			p += RadarModel::convert(obj.dynamics_vel_y_std, p);
			p += RadarModel::convert(obj.dynamics_vel_z, p);
			p += RadarModel::convert(obj.dynamics_vel_z_std, p);
			p += RadarModel::convert(obj.dynamics_acce_x, p);
			p += RadarModel::convert(obj.dynamics_acce_x_std, p);
			p += RadarModel::convert(obj.dynamics_acce_y, p);
			p += RadarModel::convert(obj.dynamics_acce_y_std, p);
			p += RadarModel::convert(obj.dynamics_acce_z, p);
			p += RadarModel::convert(obj.dynamics_acce_z_std, p);
			p += RadarModel::convert(obj.dynamics_orientation_rate, p);
			p += RadarModel::convert(obj.dynamics_orientation_rate_std, p);
			p += RadarModel::convert(obj.existence_prob, p);
			p += RadarModel::convert(obj.obstacle_prob, p);
			p += RadarModel::convert(obj.classification, p);
			p += RadarModel::convert(obj.classification_prob, p);
			p += RadarModel::convert(obj.shape_length_mean, p);
			p += RadarModel::convert(obj.shape_length_std, p);
			p += RadarModel::convert(obj.shape_width_mean, p);
			p += RadarModel::convert(obj.shape_width_std, p);
			p += RadarModel::convert(obj.shape_height_mean, p);
			p += RadarModel::convert(obj.shape_height_std, p);


			// Field range limitation
			//obj.position_x = std::clamp(obj.position_x, -200.0f, 200.0f);
			//obj.position_y = std::clamp(obj.position_y, -50.0f, 50.0f);
			//obj.size_length = std::clamp(obj.size_length, 0.1f, 10.0f);
			obj_list_msg.objectlist_objects.push_back(obj);
		}

		// Publish message
		obj_list_pub_->publish(obj_list_msg);
		RCLCPP_INFO(node_->get_logger(), "[%s] Published ObjectList: %d object",
			config_.radar_id.c_str(), obj_list_msg.objectlist_numofobjects);

		// Point cloud conversion
		auto cloud_msg = create_point_cloud(config_.frame_id, obj_list_msg.objectlist_numofobjects, node_->get_clock()->now());
		for (size_t i = 0; i < obj_list_msg.objectlist_objects.size(); ++i) {
			const auto& obj = obj_list_msg.objectlist_objects[i];
			size_t data_offset = i * cloud_msg.point_step;

			// Spherical coordinates to Cartesian coordinates
			float x = obj.position_x;
			float y = obj.position_y;
			float z = obj.position_z;

			// Intensity value
			float intensity = obj.existence_prob * 255.0f;

			// Copy data to point cloud
			memcpy(&cloud_msg.data[data_offset + 0], &x, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 4], &y, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 8], &z, sizeof(float));
			memcpy(&cloud_msg.data[data_offset + 12], &intensity, sizeof(float));
		}

		// Publish point cloud
		radar_track_pub_->publish(cloud_msg);
		RCLCPP_INFO(node_->get_logger(), "Published object_list PointCloud2 with %zu points",
			obj_list_msg.objectlist_objects.size());
	}


	void  RadarParser::parse_version_info(const std::vector<uint8_t>& data, uint64_t frame_ts)
	{
		constexpr size_t LIST_SIZE_LEN = 72;
		if (data.size() != LIST_SIZE_LEN) {
			RCLCPP_WARN(node_->get_logger(), "Version len mismatch: %zu vs %zu", data.size(), LIST_SIZE_LEN);
			return;
		}

		auto version_info = radar_msgs::msg::Version();
		version_info.header.stamp = node_->get_clock()->now();
		version_info.header.frame_id = config_.frame_id;

		uint8_t* p = (uint8_t*)data.data();
		p += RadarModel::convert(version_info.crc, p);
		p += RadarModel::convert(version_info.length, p);
		p += RadarModel::convert(version_info.sqc, p);
		p += RadarModel::convert(version_info.data_id, p);
		p += RadarModel::convert(version_info.frame_count, p);
		p += RadarModel::convert(version_info.protocol_version, p);
		p += RadarModel::convert(version_info.hw_pcb_ver, p);
		p += RadarModel::convert(version_info.hw_bom_ver, p);
		p += RadarModel::convert(version_info.cfg_version, p);
		p += RadarModel::convert(version_info.sw_pl_ver, p);
		p += RadarModel::convert(version_info.sw_ps_ver, p);
		p += RadarModel::convert(version_info.alg_pt_ver, p);
		p += RadarModel::convert(version_info.alg_trk_ver, p);
		p += RadarModel::convert(version_info.calib_ver, p);
		p += RadarModel::convert(version_info.sn_shell, p);


		ver_info_pub_->publish(version_info);
		RCLCPP_INFO(node_->get_logger(), "[%s] Published Version", config_.radar_id.c_str());

	}

	// Universal point cloud construction function
	sensor_msgs::msg::PointCloud2 RadarParser::create_point_cloud(const std::string& frame_id, size_t point_count, const rclcpp::Time& timestamp)
	{
		constexpr size_t POINT_CLOUD_FIELD_COUNT = 4;
		constexpr size_t POINT_STEP = 16;

		sensor_msgs::msg::PointCloud2 cloud_msg;
		cloud_msg.header.frame_id = frame_id;
		cloud_msg.header.stamp = timestamp;
		cloud_msg.height = 1;
		cloud_msg.width = point_count;
		cloud_msg.is_dense = true;
		cloud_msg.is_bigendian = false;
		cloud_msg.point_step = POINT_STEP;
		cloud_msg.row_step = POINT_STEP * point_count;
		cloud_msg.data.resize(cloud_msg.row_step);

		// Define fields (reuse constants)
		cloud_msg.fields.resize(POINT_CLOUD_FIELD_COUNT);

		// x field
		cloud_msg.fields[0].name = "x";
		cloud_msg.fields[0].offset = 0;
		cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud_msg.fields[0].count = 1;

		// y field
		cloud_msg.fields[1].name = "y";
		cloud_msg.fields[1].offset = 4;
		cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud_msg.fields[1].count = 1;

		// z field
		cloud_msg.fields[2].name = "z";
		cloud_msg.fields[2].offset = 8;
		cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud_msg.fields[2].count = 1;

		// intensity field
		cloud_msg.fields[3].name = "intensity";
		cloud_msg.fields[3].offset = 12;
		cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
		cloud_msg.fields[3].count = 1;

		return cloud_msg;
	}

	void RadarParser::publish_tf()
	{
		geometry_msgs::msg::TransformStamped tf_msg;
		tf_msg.header.stamp = node_->get_clock()->now();
		tf_msg.header.frame_id = config_.parent_frame_id;
		tf_msg.child_frame_id = config_.frame_id;

		config_.x = 0;
		config_.x = 0;
		config_.z = 0;

		// Translation (radar relative to parent coordinate system)
		tf_msg.transform.translation.x = config_.x;
		tf_msg.transform.translation.y = config_.y;
		tf_msg.transform.translation.z = config_.z;

		// Rotation (Euler angles → quaternion)
		tf2::Quaternion q;
		q.setRPY(config_.roll, config_.pitch, config_.yaw);
		tf_msg.transform.rotation.x = q.x();
		tf_msg.transform.rotation.y = q.y();
		tf_msg.transform.rotation.z = q.z();
		tf_msg.transform.rotation.w = q.w();

		tf_broadcaster_->sendTransform(tf_msg);
	}

}  // namespace radar_muniu