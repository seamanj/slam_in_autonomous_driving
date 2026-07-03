//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/mapping_2d.h"
#include "common/io_utils.h"
#include "common/path_utils.h"
DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_bool(with_loop_closing, false, "是否使用回环检测");

/// 测试2D lidar SLAM

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(FLAGS_bag_path);
    sad::Mapping2D mapping;


    // 获取可执行文件所在目录
    std::string exe_dir = sad::GetExecutableDir();
    if (exe_dir.empty()) {
        LOG(ERROR) << "Failed to get executable directory";
        return -1;
    }
    
    LOG(INFO) << "Executable directory: " << exe_dir;
    
    // 在可执行文件所在目录创建 data/ch6 文件夹
    std::string data_dir = exe_dir + "/data/ch6";
    std::system(("mkdir -p " + data_dir).c_str());
    std::system(("rm -rf " + data_dir + "/*").c_str());


    if (mapping.Init(FLAGS_with_loop_closing) == false) {
        return -1;
    }

    rosbag_io.AddScan2DHandle("/pavo_scan_bottom", [&](Scan2d::Ptr scan) { return mapping.ProcessScan(scan); }).Go();
    cv::imwrite(data_dir + "/global_map.png", mapping.ShowGlobalMap(2000));
    return 0;
}