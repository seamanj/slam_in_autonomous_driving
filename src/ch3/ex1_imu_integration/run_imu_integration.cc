//
// Created by xiang on 2021/11/5.
//
#include <gflags/gflags.h>  // 添加这一行
#include <glog/logging.h>
#include <iomanip>

// #include "ch3/ex1_imu_integration/imu_integration.h"
#include "imu_integration.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "./data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图形界面");

/// 本程序演示如何对IMU进行直接积分
/// 该程序需要输入data/ch3/下的文本文件，同时它将状态输出到data/ch3/state.txt中，在UI中也可以观察到车辆运动
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    sad::TxtIO io(FLAGS_imu_txt_path);

    // 该实验中，我们假设零偏已知
    Vec3d gravity(0, 0, -9.8);  // 重力方向
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    sad::IMUIntegration imu_integ(gravity, init_bg, init_ba);

    std::shared_ptr<sad::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<sad::ui::PangolinWindow>();
        ui->Init();
    }

    /// 记录结果
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    // 输出文件格式为：timestamp px py pz qw qx qy qz vx vy vz

    std::ofstream fout("./data/ch3/state.txt");

    /* common/io_utils.cc

        if (data_type == "IMU" && imu_proc_) {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
        } else if (data_type == "ODOM" && odom_proc_) {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            odom_proc_(Odom(time, wl, wr));
        } else if (data_type == "GNSS" && gnss_proc_) {
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            gnss_proc_(GNSS(time, 4, Vec3d(lat, lon, alt), heading, heading_valid));
        }
     */


    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const sad::IMU& imu) {
          imu_integ.AddIMU(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetV(), imu_integ.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavState());
              usleep(1e2);
          }
      }).Go();

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}