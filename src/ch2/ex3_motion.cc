//
// Created by xiang on 22-12-29.
// Modified: 带旋转的抛物线运动
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个带旋转的抛物线运动
/// 物体绕 Z 轴自转，同时受重力影响做平抛运动

DEFINE_double(angular_velocity, 180.0, "角速度（角度制）");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");
DEFINE_double(gravity, 9.8, "重力加速度 m/s^2");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::GLOG_INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    // 运动参数
    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                     // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量（绕Z轴自转）
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度（恒定，沿X轴方向）
    
    // 物理参数
    const Vec3d gravity(0, 0, -FLAGS_gravity);                                   // 重力加速度（世界系，向下）
    const double dt = 0.05;                                                      // 每次更新的时间步长
    
    // 状态变量
    Vec3d v_world = pose.so3() * v_body;                                         // 当前世界系速度（仅来自本体运动）
    Vec3d v_gravity(0, 0, 0);                                                   // 重力产生的速度（世界系，单独维护）

    while (ui.ShouldQuit() == false) {
        // ========== 1. 更新重力速度 ==========
        // 重力加速度累积：v = v0 + a * t
        v_gravity += gravity * dt;

        // ========== 2. 更新位置 ==========
        // 使用匀加速运动公式：p = p0 + v0 * t + 0.5 * a * t^2
        // 这里 v_world 是本体速度（世界系），v_gravity 是重力速度
        // 合速度 = v_world + v_gravity
        // 位移 = 合速度 * dt + 0.5 * gravity * dt^2
        pose.translation() += (v_world + v_gravity) * dt + 0.5 * gravity * dt * dt;

        // ========== 3. 更新旋转（绕Z轴自转） ==========
        if (FLAGS_use_quaternion) {
            // 四元数更新方式
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 
                                                         0.5 * omega[1] * dt, 
                                                         0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            // SO3 指数映射更新方式
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }

        // ========== 4. 更新本体速度的世界系表示 ==========
        // 因为旋转矩阵变了，所以 v_body 转到世界系的结果也会变
        v_world = pose.so3() * v_body;

        // ========== 5. 输出日志和可视化 ==========
        LOG(INFO) << "pose: " << pose.translation().transpose()
                  << ", v_world: " << v_world.transpose()
                  << ", v_gravity: " << v_gravity.transpose();
        
        ui.UpdateNavState(sad::NavStated(0, pose, v_world + v_gravity));

        // 等待，控制动画速度
        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}