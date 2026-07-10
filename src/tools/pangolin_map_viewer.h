//
// Created by xiang on 2022/7/19.
//

#ifndef SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_VIEWER_H
#define SLAM_IN_AUTO_DRIVING_PANGOLIN_MAP_VIEWER_H

#include <glog/logging.h>

#include <memory>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "tools/ui/pangolin_map_window.h"

namespace sad {

/// 基于 Pangolin 的局部地图查看器
class PangolinMapViewer {
public:
    /// 构造函数，指定 voxel size
    explicit PangolinMapViewer(const float& leaf_size,
                               bool use_pangolin = true)
        : leaf_size_(leaf_size),
          tmp_cloud_(new PointCloudType),
          local_map_(new PointCloudType) {

        voxel_filter_.setLeafSize(
            leaf_size_,
            leaf_size_,
            leaf_size_);

        if (use_pangolin) {

            viewer_ =
                std::make_shared<ui::PangolinMapWindow>();

            if (!viewer_->Init()) {
                LOG(ERROR)
                    << "Failed to initialize PangolinMapWindow.";
                viewer_.reset();
            }
        }
    }

    ~PangolinMapViewer() = default;

    /**
     * 增加一个 Pose 和它对应的世界系点云
     */
    void SetPoseAndCloud(
        const SE3& pose,
        CloudPtr cloud_world) {

        voxel_filter_.setInputCloud(cloud_world);
        voxel_filter_.filter(*tmp_cloud_);

        *local_map_ += *tmp_cloud_;

        voxel_filter_.setInputCloud(local_map_);
        voxel_filter_.filter(*local_map_);

        if (viewer_) {
            viewer_->UpdateMap(local_map_, pose);
        }

        if (local_map_->size() > 600000) {

            leaf_size_ *= 1.26f;

            voxel_filter_.setLeafSize(
                leaf_size_,
                leaf_size_,
                leaf_size_);

            LOG(INFO)
                << "viewer set leaf size to "
                << leaf_size_;
        }
    }

    /// 保存地图
    void SaveMap(const std::string& path) 
    {
        if (local_map_->size() > 0) {
            sad::SaveCloudToFile(path, *local_map_);
            LOG(INFO) << "save map to " << path;
        } else {
            LOG(INFO) << "map 是空的" << path;
        }
    }

    void Clean() {
        tmp_cloud_->clear();
        local_map_->clear();
    }

    void ClearAndResetLeafSize(const float leaf_size) 
    {

        leaf_size_ = leaf_size;
        tmp_cloud_->clear();
        local_map_->clear();
        voxel_filter_.setLeafSize(
            leaf_size_,
            leaf_size_,
            leaf_size_);
    }

    CloudPtr GetLocalMap() const {
        return local_map_;
    }

    bool ShouldQuit() const {

        if (!viewer_) {
            return true;
        }

        return viewer_->ShouldQuit();
    }

private:
    pcl::VoxelGrid<PointType> voxel_filter_;

    std::shared_ptr<ui::PangolinMapWindow> viewer_ = nullptr;

    float leaf_size_ = 1.0f;

    CloudPtr tmp_cloud_;

    CloudPtr local_map_;
};

}  // namespace sad

#endif