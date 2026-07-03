//
// Created by xiang on 2021/7/19.
//

#include "gins_pre_integ_ceres.h"
#include "ceres_types.h"

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <glog/logging.h>

namespace sad {

// 自定义 PoseManifold: 7维参数块 [x, y, z, qx, qy, qz, qw]
// 其中前3维是平移（欧几里得），后4维是四元数（SO3）
class PoseManifold : public ceres::Manifold {
public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }  // 3 平移 + 3 旋转

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        // x: [x, y, z, qx, qy, qz, qw]
        // delta: [dx, dy, dz, dqx, dqy, dqz] (6维)
        
        // 平移部分: 直接加
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];
        
        // 四元数部分: 使用增量旋转
        Eigen::Quaterniond q(x[6], x[3], x[4], x[5]);  // w, x, y, z
        
        // 构造增量四元数 (delta[3], delta[4], delta[5] 是旋转向量)
        Eigen::Vector3d omega(delta[3], delta[4], delta[5]);
        double theta = omega.norm();
        Eigen::Quaterniond dq;
        if (theta < 1e-10) {
            dq = Eigen::Quaterniond::Identity();
        } else {
            Eigen::Vector3d axis = omega / theta;
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
        }
        
        // 应用旋转
        Eigen::Quaterniond q_new = dq * q;
        q_new.normalize();
        
        x_plus_delta[3] = q_new.x();
        x_plus_delta[4] = q_new.y();
        x_plus_delta[5] = q_new.z();
        x_plus_delta[6] = q_new.w();
        
        return true;
    }

    bool Minus(const double* y, const double* x, double* y_minus_x) const override {
        // y - x 在切空间中的表示
        // 平移部分: 直接减
        y_minus_x[0] = y[0] - x[0];
        y_minus_x[1] = y[1] - x[1];
        y_minus_x[2] = y[2] - x[2];
        
        // 旋转部分: 计算相对旋转
        Eigen::Quaterniond qx(x[6], x[3], x[4], x[5]);
        Eigen::Quaterniond qy(y[6], y[3], y[4], y[5]);
        
        Eigen::Quaterniond dq = qy * qx.inverse();
        dq.normalize();
        
        // 转换为轴角
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d omega = aa.angle() * aa.axis();
        
        y_minus_x[3] = omega.x();
        y_minus_x[4] = omega.y();
        y_minus_x[5] = omega.z();
        
        return true;
    }

    // PlusJacobian: 计算 Plus 操作关于 x 和 delta 的雅可比矩阵
    bool PlusJacobian(const double* x, double* jacobian) const override {
        // jacobian 是 7x6 的矩阵 (行优先)
        // 对于平移部分: 雅可比是单位矩阵
        // 对于旋转部分: 雅可比是 QuaternionParameterization 的雅可比
        
        // 初始化雅可比为零
        std::fill(jacobian, jacobian + 7 * 6, 0.0);
        
        // 平移部分: 前3行，前3列是单位矩阵
        for (int i = 0; i < 3; ++i) {
            jacobian[i * 6 + i] = 1.0;
        }
        
        // 旋转部分: 后4行，后3列
        // 使用四元数的左乘雅可比
        Eigen::Quaterniond q(x[6], x[3], x[4], x[5]);
        
        // 计算 d(q * dq) / d(dq) 在 dq = 0 处的雅可比
        // 这是 4x3 的矩阵
        Eigen::Matrix<double, 4, 3> quat_jacobian;
        quat_jacobian.setZero();
        
        // 对于四元数 q = [w, x, y, z]，左乘增量旋转的雅可比
        // 在 dq = [0, 0, 0, 1] 处 (单位四元数)
        // d(q * dq)/d(dq_rot) 的雅可比
        const double w = q.w();
        const double x_q = q.x();
        const double y_q = q.y();
        const double z_q = q.z();
        
        // 左乘四元数的雅可比 (4x3)
        // 参考: https://github.com/ceres-solver/ceres-solver/issues/696
        quat_jacobian(0, 0) = -0.5 * x_q;  // d(w)/d(dx)
        quat_jacobian(0, 1) = -0.5 * y_q;  // d(w)/d(dy)
        quat_jacobian(0, 2) = -0.5 * z_q;  // d(w)/d(dz)
        
        quat_jacobian(1, 0) = 0.5 * w;     // d(x_q)/d(dx)
        quat_jacobian(1, 1) = 0.5 * z_q;   // d(x_q)/d(dy)
        quat_jacobian(1, 2) = -0.5 * y_q;  // d(x_q)/d(dz)
        
        quat_jacobian(2, 0) = -0.5 * z_q;  // d(y_q)/d(dx)
        quat_jacobian(2, 1) = 0.5 * w;     // d(y_q)/d(dy)
        quat_jacobian(2, 2) = 0.5 * x_q;   // d(y_q)/d(dz)
        
        quat_jacobian(3, 0) = 0.5 * y_q;   // d(z_q)/d(dx)
        quat_jacobian(3, 1) = -0.5 * x_q;  // d(z_q)/d(dy)
        quat_jacobian(3, 2) = 0.5 * w;     // d(z_q)/d(dz)
        
        // 将四元数雅可比放入总雅可比的旋转部分 (后4行，后3列)
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                jacobian[(3 + i) * 6 + (3 + j)] = quat_jacobian(i, j);
            }
        }
        
        return true;
    }

    // MinusJacobian: 计算 Minus 操作关于 x 的雅可比矩阵
    bool MinusJacobian(const double* x, double* jacobian) const override {
        // jacobian 是 6x7 的矩阵 (行优先)
        // 对于平移部分: 雅可比是单位矩阵
        // 对于旋转部分: 雅可比是 QuaternionParameterization 的雅可比
        
        std::fill(jacobian, jacobian + 6 * 7, 0.0);
        
        // 平移部分: 前3行，前3列是单位矩阵
        for (int i = 0; i < 3; ++i) {
            jacobian[i * 7 + i] = 1.0;
        }
        
        // 旋转部分: 后3行，后4列
        Eigen::Quaterniond q(x[6], x[3], x[4], x[5]);
        
        // 计算 d(log(q * qx^{-1})) / d(qx) 的雅可比
        // 这是 3x4 的矩阵
        Eigen::Matrix<double, 3, 4> quat_jacobian;
        quat_jacobian.setZero();
        
        const double w = q.w();
        const double x_q = q.x();
        const double y_q = q.y();
        const double z_q = q.z();
        
        // Minus 的雅可比是 Plus 雅可比的逆
        // 对于四元数，这个雅可比是：
        quat_jacobian(0, 0) = -x_q;  // d(dx)/d(w)
        quat_jacobian(0, 1) = w;     // d(dx)/d(x)
        quat_jacobian(0, 2) = z_q;   // d(dx)/d(y)
        quat_jacobian(0, 3) = -y_q;  // d(dx)/d(z)
        
        quat_jacobian(1, 0) = -y_q;  // d(dy)/d(w)
        quat_jacobian(1, 1) = -z_q;  // d(dy)/d(x)
        quat_jacobian(1, 2) = w;     // d(dy)/d(y)
        quat_jacobian(1, 3) = x_q;   // d(dy)/d(z)
        
        quat_jacobian(2, 0) = -z_q;  // d(dz)/d(w)
        quat_jacobian(2, 1) = y_q;   // d(dz)/d(x)
        quat_jacobian(2, 2) = -x_q;  // d(dz)/d(y)
        quat_jacobian(2, 3) = w;     // d(dz)/d(z)
        
        // 将四元数雅可比放入总雅可比的旋转部分 (后3行，后4列)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                jacobian[(3 + i) * 7 + (3 + j)] = quat_jacobian(i, j);
            }
        }
        
        return true;
    }
};

void GinsPreInteg::AddImu(const IMU& imu) {
    if (first_gnss_received_ && first_imu_received_) {
        pre_integ_->Integrate(last_imu_, imu.timestamp_ - last_imu_.timestamp_);
    }

    first_imu_received_ = true;
    last_imu_ = imu;
    current_time_ = imu.timestamp_;
}

void GinsPreInteg::SetOptions(sad::GinsPreInteg::Options options) {
    double bg_rw2 = 1.0 / (options_.bias_gyro_var_ * options_.bias_gyro_var_);
    options_.bg_rw_info_.diagonal() << bg_rw2, bg_rw2, bg_rw2;
    double ba_rw2 = 1.0 / (options_.bias_acce_var_ * options_.bias_acce_var_);
    options_.ba_rw_info_.diagonal() << ba_rw2, ba_rw2, ba_rw2;

    double gp2 = options_.gnss_pos_noise_ * options_.gnss_pos_noise_;
    double gh2 = options_.gnss_height_noise_ * options_.gnss_height_noise_;
    double ga2 = options_.gnss_ang_noise_ * options_.gnss_ang_noise_;

    options_.gnss_info_.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gh2;
    pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);

    double o2 = 1.0 / (options_.odom_var_ * options_.odom_var_);
    options_.odom_info_.diagonal() << o2, o2, o2;

    prior_info_.block<6, 6>(9, 9) = Mat6d ::Identity() * 1e6;

    if (this_frame_) {
        this_frame_->bg_ = options_.preinteg_options_.init_bg_;
        this_frame_->ba_ = options_.preinteg_options_.init_ba_;
    }
}

void GinsPreInteg::AddGnss(const GNSS& gnss) {
    this_frame_ = std::make_shared<NavStated>(current_time_); 
    this_gnss_ = gnss;

    if (!first_gnss_received_) {
        if (!gnss.heading_valid_) {
            return;
        }

        this_frame_->timestamp_ = gnss.unix_time_;
        this_frame_->p_ = gnss.utm_pose_.translation();
        this_frame_->R_ = gnss.utm_pose_.so3();
        this_frame_->v_.setZero();
        this_frame_->bg_ = options_.preinteg_options_.init_bg_;
        this_frame_->ba_ = options_.preinteg_options_.init_ba_;

        pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);

        last_frame_ = this_frame_;
        last_gnss_ = this_gnss_;
        first_gnss_received_ = true;
        current_time_ = gnss.unix_time_;
        return;
    }

    pre_integ_->Integrate(last_imu_, gnss.unix_time_ - current_time_);

    current_time_ = gnss.unix_time_;
    *this_frame_ = pre_integ_->Predict(*last_frame_, options_.gravity_);

    Optimize();

    last_imu_.timestamp_ = this_frame_->timestamp_;

    last_frame_ = this_frame_;
    last_gnss_ = this_gnss_;
}

void GinsPreInteg::AddOdom(const sad::Odom& odom) {
    last_odom_ = odom;
    last_odom_set_ = true;
}

void GinsPreInteg::Optimize() {
    if (pre_integ_->dt_ < 1e-3) {
        return;
    }

    auto load_pose = [](const NavStated& state, double* pose) {
        const auto q = state.R_.unit_quaternion();
        pose[0] = state.p_[0];
        pose[1] = state.p_[1];
        pose[2] = state.p_[2];
        pose[3] = q.x();
        pose[4] = q.y();
        pose[5] = q.z();
        pose[6] = q.w();
    };
    auto write_pose = [](const double* pose, NavStated& state) {
        state.p_ = Vec3d(pose[0], pose[1], pose[2]);
        Quatd q(pose[6], pose[3], pose[4], pose[5]);
        q.normalize();
        state.R_ = SO3(q);
    };

    double pose0[7], pose1[7];
    double vel0[3], vel1[3], bg0[3], bg1[3], ba0[3], ba1[3];

    load_pose(*last_frame_, pose0);
    load_pose(*this_frame_, pose1);

    for (int i = 0; i < 3; ++i) {
        vel0[i] = last_frame_->v_[i];
        vel1[i] = this_frame_->v_[i];
        bg0[i] = last_frame_->bg_[i];
        bg1[i] = this_frame_->bg_[i];
        ba0[i] = last_frame_->ba_[i];
        ba1[i] = this_frame_->ba_[i];
    }

    ceres::Problem problem;

    problem.AddParameterBlock(pose0, 7);
    problem.AddParameterBlock(pose1, 7);
    
    // 使用自定义的 PoseManifold
    problem.SetManifold(pose0, new PoseManifold());
    problem.SetManifold(pose1, new PoseManifold());
    
    problem.AddParameterBlock(vel0, 3);
    problem.AddParameterBlock(vel1, 3);
    problem.AddParameterBlock(bg0, 3);
    problem.AddParameterBlock(bg1, 3);
    problem.AddParameterBlock(ba0, 3);
    problem.AddParameterBlock(ba1, 3);

    auto* inertial_cost =
        new ceres::AutoDiffCostFunction<InertialResidual, 15, 7, 3, 3, 3, 7, 3, 3, 3>(
            new InertialResidual(pre_integ_, options_.gravity_));
    problem.AddResidualBlock(inertial_cost, new ceres::HuberLoss(200.0), pose0, vel0, bg0, ba0, pose1, vel1, bg1,
                             ba1);

    auto* gyro_rw_cost =
        new ceres::AutoDiffCostFunction<GyroRWResidual, 3, 3, 3>(new GyroRWResidual(options_.bg_rw_info_));
    problem.AddResidualBlock(gyro_rw_cost, nullptr, bg0, bg1);

    auto* acc_rw_cost =
        new ceres::AutoDiffCostFunction<AccRWResidual, 3, 3, 3>(new AccRWResidual(options_.ba_rw_info_));
    problem.AddResidualBlock(acc_rw_cost, nullptr, ba0, ba1);

    auto* prior_cost =
        new ceres::AutoDiffCostFunction<PriorResidual, 15, 7, 3, 3, 3>(new PriorResidual(*last_frame_, prior_info_));
    problem.AddResidualBlock(prior_cost, nullptr, pose0, vel0, bg0, ba0);

    auto* gnss0_cost =
        new ceres::AutoDiffCostFunction<GNSSResidual, 6, 7>(new GNSSResidual(last_gnss_.utm_pose_, options_.gnss_info_));
    problem.AddResidualBlock(gnss0_cost, nullptr, pose0);

    auto* gnss1_cost =
        new ceres::AutoDiffCostFunction<GNSSResidual, 6, 7>(new GNSSResidual(this_gnss_.utm_pose_, options_.gnss_info_));
    problem.AddResidualBlock(gnss1_cost, nullptr, pose1);

    Vec3d vel_odom = Vec3d::Zero();
    if (last_odom_set_) {
        double velo_l =
            options_.wheel_radius_ * last_odom_.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
        double velo_r =
            options_.wheel_radius_ * last_odom_.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
        double average_vel = 0.5 * (velo_l + velo_r);
        vel_odom = Vec3d(average_vel, 0.0, 0.0);

        auto* odom_cost =
            new ceres::AutoDiffCostFunction<OdomResidual, 3, 3, 7>(new OdomResidual(vel_odom, options_.odom_info_));
        problem.AddResidualBlock(odom_cost, nullptr, vel1, pose1);

        last_odom_set_ = false;
    }

    ceres::Solver::Options ceres_options;
    ceres_options.linear_solver_type = ceres::DENSE_QR;
    ceres_options.max_num_iterations = 20;
    ceres_options.minimizer_progress_to_stdout = options_.verbose_;

    ceres::Solver::Summary summary;
    ceres::Solve(ceres_options, &problem, &summary);
    if (options_.verbose_) {
        LOG(INFO) << summary.BriefReport();
    }

    write_pose(pose0, *last_frame_);
    write_pose(pose1, *this_frame_);

    for (int i = 0; i < 3; ++i) {
        last_frame_->v_[i] = vel0[i];
        this_frame_->v_[i] = vel1[i];
        last_frame_->bg_[i] = bg0[i];
        this_frame_->bg_[i] = bg1[i];
        last_frame_->ba_[i] = ba0[i];
        this_frame_->ba_[i] = ba1[i];
    }

    options_.preinteg_options_.init_bg_ = this_frame_->bg_;
    options_.preinteg_options_.init_ba_ = this_frame_->ba_;
    pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_);
}

NavStated GinsPreInteg::GetState() const {
    if (this_frame_ == nullptr) {
        return {};
    }

    if (pre_integ_ == nullptr) {
        return *this_frame_;
    }

    return pre_integ_->Predict(*this_frame_, options_.gravity_);
}

}  // namespace sad