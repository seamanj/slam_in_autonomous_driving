//
// Created by xiang on 2021/8/9.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include <libgen.h>  // for dirname
#include <unistd.h>  // for readlink
#include <cstring>   // for strcpy
#include "common/path_utils.h"
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "./data/ch5/scan_example.pcd", "点云文件路径");
DEFINE_double(azimuth_resolution_deg, 0.3, "方位角分辨率（度）");
DEFINE_int32(elevation_rows, 16, "俯仰角对应的行数");
DEFINE_double(elevation_range, 15.0, "俯仰角范围");  // VLP-16 上下各15度范围
DEFINE_double(lidar_height, 1.128, "雷达安装高度");
DEFINE_string(output_path, "", "输出图像路径（可选，默认保存到可执行文件目录）");


std::string getOutputPath() {
    // 如果用户指定了输出路径，直接使用
    if (!FLAGS_output_path.empty()) {
        return FLAGS_output_path;
    }
    
    // 否则保存到可执行文件所在目录
    std::string exe_path = sad::GetExecutablePath();
    if (exe_path.empty()) {
        LOG(WARNING) << "Failed to get executable path, using current directory";
        return "./range_image.png";
    }
    
    // 复制一份因为 dirname 可能会修改原字符串
    char* exe_path_cstr = strdup(exe_path.c_str());
    std::string exe_dir = dirname(exe_path_cstr);
    free(exe_path_cstr);
    
    return exe_dir + "/range_image.png";
}

void GenerateRangeImage(PointCloudType::Ptr cloud, const std::string& output_path) {
    int image_cols = int(360 / FLAGS_azimuth_resolution_deg);  // 水平为360度，按分辨率切分即可
    int image_rows = FLAGS_elevation_rows;                     // 图像行数固定
    LOG(INFO) << "range image: " << image_rows << "x" << image_cols;
    
    // 检查图像尺寸
    if (image_rows <= 0 || image_cols <= 0) {
        LOG(ERROR) << "Invalid image dimensions: " << image_rows << "x" << image_cols;
        return;
    }

    // 我们生成一个HSV图像以更好地显示图像
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(0, 0, 0));

    double ele_resolution = FLAGS_elevation_range * 2 / FLAGS_elevation_rows;  // elevation分辨率

    int valid_points = 0;
    for (const auto& pt : cloud->points) {
        double azimuth = atan2(pt.y, pt.x) * 180 / M_PI;
        double range = sqrt(pt.x * pt.x + pt.y * pt.y + 
                           (pt.z - FLAGS_lidar_height) * (pt.z - FLAGS_lidar_height));
        double elevation = asin((pt.z - FLAGS_lidar_height) / range) * 180 / M_PI;

        // keep in 0~360
        if (azimuth < 0) {
            azimuth += 360;
        }

        int x = int(azimuth / FLAGS_azimuth_resolution_deg);                      // 列
        int y = int((elevation + FLAGS_elevation_range) / ele_resolution + 0.5);  // 行

        if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
            // 使用距离信息作为Hue值，Saturation固定为255，Value固定为127
            // 距离越远颜色越红（HSV中红色对应0度或360度）
            uchar hue = uchar(std::min(179.0, (range / 100.0) * 179.0));  // OpenCV中Hue范围是0-179
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(hue, 255, 127);
            valid_points++;
        }
    }

    LOG(INFO) << "Valid points mapped to range image: " << valid_points << " / " << cloud->size();

    // 沿Y轴翻转，因为我们希望Z轴朝上时Y朝上
    cv::Mat image_flipped;
    cv::flip(image, image_flipped, 0);

    // hsv to rgb
    cv::Mat image_rgb;
    cv::cvtColor(image_flipped, image_rgb, cv::COLOR_HSV2BGR);
    
    // 保存图像
    if (cv::imwrite(output_path, image_rgb)) {
        LOG(INFO) << "Range image saved to: " << output_path;
    } else {
        LOG(ERROR) << "Failed to save range image to: " << output_path;
    }
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

    // 获取输出路径
    std::string output_path = getOutputPath();
    LOG(INFO) << "Output path: " << output_path;

    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud) < 0) {
        LOG(ERROR) << "Failed to load PCD file: " << FLAGS_pcd_path;
        return -1;
    }

    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points: " << cloud->size();
    GenerateRangeImage(cloud, output_path);

    return 0;
} 