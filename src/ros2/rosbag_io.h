//
// Created by xiang on 2021/7/20.
//

#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H

// ROS 2 头文件
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>


#include <fstream>
#include <functional>
#include <utility>
#include <memory>

#include "common/dataset_type.h"
#include "common/global_flags.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/lidar_utils.h"
#include "common/math_utils.h"
#include "common/message_def.h"
#include "common/odom.h"
// #include "livox_ros_driver/msg/custom_msg.hpp"
// #include "tools/pointcloud_convert/velodyne_convertor.h"

#include "ch3/utm_convert.h"


// 在 io_utils.h 中修改
using MessageProcessFunction = std::function<bool(std::shared_ptr<rclcpp::SerializedMessage>, const std::string &)>;

namespace sad {


/**
 * ROSBAG2 IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        assert(dataset_type_ != DatasetType::UNKNOWN);
    }

    // ROS 2 中消息类型定义
    using Scan2DMsg = sensor_msgs::msg::LaserScan;
    using MultiScan2DMsg = sensor_msgs::msg::MultiEchoLaserScan;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
    using ImuMsg = sensor_msgs::msg::Imu;
    // using LivoxCustomMsg = livox_ros_driver::msg::CustomMsg;
    // using VelodynePacketMsg = velodyne_msgs::msg::VelodynePacket;
    // using VelodyneScanMsg = velodyne_msgs::msg::VelodyneScan;

    // 回调函数类型定义
    using MessageProcessFunction = std::function<bool(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic)>;
    
    // 简化的回调函数类型
    using Scan2DHandle = std::function<bool(Scan2DMsg::SharedPtr)>;
    using MultiScan2DHandle = std::function<bool(MultiScan2DMsg::SharedPtr)>;
    using PointCloud2Handle = std::function<bool(PointCloud2Msg::SharedPtr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using GNSSHandle = std::function<bool(GNSSPtr)>;
    using OdomHandle = std::function<bool(const Odom &)>;
    // using LivoxHandle = std::function<bool(LivoxCustomMsg::SharedPtr)>;
    // using VelodyneHandle = std::function<bool(VelodyneScanMsg::SharedPtr)>;

    // 遍历文件内容，调用回调函数
    void Go();

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// 2D激光处理
    RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
        return AddHandle(topic_name, [f](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
            auto deserialized_msg = std::make_shared<Scan2DMsg>();
            // 反序列化消息
            rclcpp::Serialization<Scan2DMsg> serialization;
            serialization.deserialize_message(msg.get(), deserialized_msg.get());
            return f(deserialized_msg);
        });
    }

    /// 多回波2D激光处理
    RosbagIO &AddMultiScan2DHandle(const std::string &topic_name, MultiScan2DHandle f) {
        return AddHandle(topic_name, [f](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
            auto deserialized_msg = std::make_shared<MultiScan2DMsg>();
            rclcpp::Serialization<MultiScan2DMsg> serialization;
            serialization.deserialize_message(msg.get(), deserialized_msg.get());
            return f(deserialized_msg);
        });
    }

    /// 根据数据集类型自动确定topic名称
    RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f) {
        // if (dataset_type_ == DatasetType::WXB_3D) {
        //     return AddHandle(wxb_lidar_topic, [f, this](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
        //         auto deserialized_msg = std::make_shared<VelodyneScanMsg>();
        //         rclcpp::Serialization<VelodyneScanMsg> serialization;
        //         serialization.deserialize_message(msg.get(), deserialized_msg.get());

        //         FullCloudPtr cloud(new FullPointCloudType), cloud_out(new FullPointCloudType);
        //         vlp_parser_.ProcessScan(deserialized_msg, cloud);
        //         PointCloud2Msg::SharedPtr cloud_msg(new PointCloud2Msg);
        //         pcl::toROSMsg(*cloud, *cloud_msg);
        //         return f(cloud_msg);
        //     });
        // } else 
        if (dataset_type_ == DatasetType::AVIA) {
            // AVIA 不能直接获取point cloud 2
            return *this;
        } else {
            return AddHandle(GetLidarTopicName(), [f](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
                auto deserialized_msg = std::make_shared<PointCloud2Msg>();
                rclcpp::Serialization<PointCloud2Msg> serialization;
                serialization.deserialize_message(msg.get(), deserialized_msg.get());
                return f(deserialized_msg);
            });
        }
    }

    /// 根据数据集自动处理RTK消息
    RosbagIO &AddAutoRTKHandle(GNSSHandle f) {
        if (dataset_type_ == DatasetType::NCLT) {
            return AddHandle(nclt_rtk_topic, [f, this](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
                auto navsat_msg = std::make_shared<NavSatFixMsg>();
                rclcpp::Serialization<NavSatFixMsg> serialization;
                serialization.deserialize_message(msg.get(), navsat_msg.get());

                GNSSPtr gnss(new GNSS(navsat_msg));
                ConvertGps2UTMOnlyTrans(*gnss);
                if (std::isnan(gnss->lat_lon_alt_[2])) {
                    // 貌似有Nan
                    return false;
                }

                return f(gnss);
            });
        } else {
            // TODO 其他数据集的RTK转换关系
            return *this;
        }
    }

    /// point cloud 2 的处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
            auto deserialized_msg = std::make_shared<PointCloud2Msg>();
            rclcpp::Serialization<PointCloud2Msg> serialization;
            serialization.deserialize_message(msg.get(), deserialized_msg.get());
            return f(deserialized_msg);
        });
    }

    /// livox msg 处理
    // RosbagIO &AddLivoxHandle(LivoxHandle f) {
    //     return AddHandle(GetLidarTopicName(), [f, this](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
    //         auto livox_msg = std::make_shared<LivoxCustomMsg>();
    //         rclcpp::Serialization<LivoxCustomMsg> serialization;
    //         serialization.deserialize_message(msg.get(), livox_msg.get());
    //         return f(livox_msg);
    //     });
    // }

    /// wxb的velodyne packets处理
    // RosbagIO &AddVelodyneHandle(const std::string &topic_name, FullPointCloudHandle f) {
    //     return AddHandle(topic_name, [f, this](std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string &topic) -> bool {
    //         auto velodyne_msg = std::make_shared<VelodyneScanMsg>();
    //         rclcpp::Serialization<VelodyneScanMsg> serialization;
    //         serialization.deserialize_message(msg.get(), velodyne_msg.get());

    //         FullCloudPtr cloud(new FullPointCloudType), cloud_out(new FullPointCloudType);
    //         vlp_parser_.ProcessScan(velodyne_msg, cloud);

    //         return f(cloud);
    //     });
    // }

    /// IMU
    RosbagIO &AddImuHandle(ImuHandle f);

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    /// 根据设定的数据集名称获取雷达名
    std::string GetLidarTopicName() const;

    /// 根据数据集名称确定IMU topic名称
    std::string GetIMUTopicName() const;

    std::map<std::string, MessageProcessFunction> process_func_;
    std::string bag_file_;
    DatasetType dataset_type_;

    // packets driver
    // tools::VelodyneConvertor vlp_parser_;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_IO_UTILS_H