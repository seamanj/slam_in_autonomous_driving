//
// Created by xiang on 2022/3/15.
//

#ifndef SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H
#define SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H

// 标准库头文件（必须在 ROS 2 头文件之前）
#include <cstring>
#include <memory>

// ROS 2 头文件
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/point_types.h"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

// 类型定义（改为 ROS 2 的消息类型）
using Scan2d = sensor_msgs::msg::LaserScan;
using MultiScan2d = sensor_msgs::msg::MultiEchoLaserScan;
using PacketsMsg = velodyne_msgs::msg::VelodyneScan;
using PacketsMsgPtr = std::shared_ptr<PacketsMsg>;

namespace sad {

inline Scan2d::SharedPtr MultiToScan2d(MultiScan2d::SharedPtr mscan) {
    Scan2d::SharedPtr scan(new Scan2d);
    scan->header = mscan->header;
    scan->range_max = mscan->range_max;
    scan->range_min = mscan->range_min;
    scan->angle_increment = mscan->angle_increment;
    scan->angle_max = mscan->angle_max;
    scan->angle_min = mscan->angle_min;
    for (auto r : mscan->ranges) {
        if (r.echoes.empty()) {
            scan->ranges.emplace_back(scan->range_max + 1.0);
        } else {
            scan->ranges.emplace_back(r.echoes[0]);
        }
    }
    for (auto i : mscan->intensities) {
        if (i.echoes.empty()) {
            scan->intensities.emplace_back(0);
        } else {
            scan->intensities.emplace_back(i.echoes[0]);
        }
    }
    scan->scan_time = mscan->scan_time;
    scan->time_increment = mscan->time_increment;

    // limit range max
    scan->range_max = 20.0;
    return scan;
}

/// ROS PointCloud2 转通常的pcl PointCloud
inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

/**
 * 其他类型点云转到PointType点云
 * 用的最多的是全量点云转到XYZI点云
 * @tparam PointT
 * @param input
 * @return
 */
template <typename PointT = FullPointType>
CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
    CloudPtr cloud(new PointCloudType);
    for (auto& pt : input->points) {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        cloud->points.template emplace_back(p);
    }
    cloud->width = input->width;
    return cloud;
}

/// 对点云进行voxel filter,指定分辨率
inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
    return value.array().template round().template cast<int>();
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_LIDAR_UTILS_H