//
// Created by xiang on 2021/7/20.
//
#include "rosbag_io.h"

#include <glog/logging.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sad {


std::string RosbagIO::GetLidarTopicName() const {
    if (dataset_type_ == DatasetType::NCLT) {
        return nclt_lidar_topic;
    }
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_lidar_topic;
    }
    if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_lidar_topic;
    }
    if (dataset_type_ == DatasetType::UTBM) {
        return utbm_lidar_topic;
    }
    if (dataset_type_ == DatasetType::AVIA) {
        return avia_lidar_topic;
    }
    return "";
}

void RosbagIO::Go() {
    LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    
    try {
        reader.open(storage_options, converter_options);
    } catch (const std::exception& e) {
        LOG(ERROR) << "cannot open " << bag_file_ << ": " << e.what();
        return;
    }

    // 创建序列化消息对象
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();

    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        auto topic_name = bag_message->topic_name;
        
        // 复制数据到 SerializedMessage
        serialized_msg->reserve(bag_message->serialized_data->buffer_length);
        memcpy(serialized_msg->get_rcl_serialized_message().buffer, 
               bag_message->serialized_data->buffer,
               bag_message->serialized_data->buffer_length);
        serialized_msg->get_rcl_serialized_message().buffer_length = 
            bag_message->serialized_data->buffer_length;

        auto iter = process_func_.find(topic_name);
        if (iter != process_func_.end()) {
            iter->second(serialized_msg, topic_name);
        }

        if (global::FLAG_EXIT) {
            break;
        }
        
        // 重置消息以便下次使用
        serialized_msg->get_rcl_serialized_message().buffer_length = 0;
    }

    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

std::string RosbagIO::GetIMUTopicName() const {
    if (dataset_type_ == DatasetType::ULHK) {
        return ulhk_imu_topic;
    } else if (dataset_type_ == DatasetType::UTBM) {
        return utbm_imu_topic;
    } else if (dataset_type_ == DatasetType::NCLT) {
        return nclt_imu_topic;
    } else if (dataset_type_ == DatasetType::WXB_3D) {
        return wxb_imu_topic;
    } else if (dataset_type_ == DatasetType::AVIA) {
        return avia_imu_topic;
    } else {
        LOG(ERROR) << "cannot load imu topic name of dataset " << int(dataset_type_);
    }

    return "";
}

// ========== 添加 AddImuHandle 的实现 ==========
RosbagIO& RosbagIO::AddImuHandle(ImuHandle f) {
    std::string imu_topic = GetIMUTopicName();
    
    // 如果无法从dataset_type获取topic，记录错误
    if (imu_topic.empty()) {
        LOG(ERROR) << "Cannot find IMU topic for dataset type: " << int(dataset_type_);
        return *this;
    }
    
    return AddHandle(imu_topic, [f](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
        auto imu_msg = std::make_shared<ImuMsg>();
        rclcpp::Serialization<ImuMsg> serialization;
        serialization.deserialize_message(msg.get(), imu_msg.get());
        
        // 将 sensor_msgs::msg::Imu 转换为 sad::IMUPtr
        IMUPtr imu_ptr = std::make_shared<IMU>();
        imu_ptr->timestamp_ = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
        
        // 角速度 (rad/s)
        imu_ptr->gyro_ = Vec3d(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
        
        // 加速度 (m/s^2)
        imu_ptr->acce_ = Vec3d(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
        
        return f(imu_ptr);
    });
}

}  // namespace sad