#pragma once

#include "common/lidar_utils.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

namespace sad {

inline CloudPtr PointCloud2ToCloudPtr(
    sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);

    return cloud;
}

}  // namespace sad