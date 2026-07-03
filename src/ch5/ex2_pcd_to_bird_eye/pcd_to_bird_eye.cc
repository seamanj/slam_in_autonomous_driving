//
// Created by xiang on 2021/8/9.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libgen.h>  // for dirname
#include <unistd.h>  // for readlink
#include <cstring>   // for strcpy
#include "common/path_utils.h"
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/map_example.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, 0.2, "俯视图最低高度");
DEFINE_double(max_z, 2.5, "俯视图最高高度");

/// 本节演示如何将一个点云转换为俯视图像
void GenerateBEVImage(PointCloudType::Ptr cloud, std::string output_path) {
    // 计算点云边界
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.y < p2.y; });
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    const double inv_r = 1.0 / FLAGS_image_resolution;

    const int image_rows = int((max_y - min_y) * inv_r);
    const int image_cols = int((max_x - min_x) * inv_r);
    
    // 检查图像尺寸
    if (image_rows <= 0 || image_cols <= 0) {
        LOG(ERROR) << "Invalid image dimensions: " << image_rows << " x " << image_cols;
        return;
    }

    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    float x_center_image = image_cols / 2.0f;
    float y_center_image = image_rows / 2.0f;

    // 生成图像
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points) {
        int x = int((pt.x - x_center) * inv_r + x_center_image);
        int y = int((pt.y - y_center) * inv_r + y_center_image);
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }

    cv::imwrite(output_path, image);
    LOG(INFO) << "BEV image saved to: " << output_path;
}


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 获取可执行文件所在目录（安全的方式）
    std::string exe_path = sad::GetExecutablePath();
    if (exe_path.empty()) {
        LOG(ERROR) << "Failed to get executable path";
        return -1;
    }
    
    // 复制一份因为 dirname 可能会修改原字符串
    char* exe_path_cstr = strdup(exe_path.c_str());
    std::string exe_dir = dirname(exe_path_cstr);
    free(exe_path_cstr);
    
    std::string output_path = exe_dir + "/bev.png";
    
    LOG(INFO) << "Executable directory: " << exe_dir;
    LOG(INFO) << "Output path: " << output_path;

    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud) < 0) {
        LOG(ERROR) << "Failed to load PCD file: " << FLAGS_pcd_path;
        return -1;
    }

    if (cloud->empty()) {
        LOG(ERROR) << "Cloud is empty";
        return -1;
    }

    LOG(INFO) << "Cloud points: " << cloud->size();
    GenerateBEVImage(cloud, output_path);

    return 0;
}