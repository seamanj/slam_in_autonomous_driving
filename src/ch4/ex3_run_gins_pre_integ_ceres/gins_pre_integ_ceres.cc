//
// Created by xiang on 2021/7/19.
//

#include "gins_pre_integ_ceres.h"
#include "ceres_types.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

namespace sad {

void GinsPreInteg::AddImu(const IMU& imu) {
    if (first_gnss_received_ && first_imu_received_) {
        // pre_integ_->Integrate(imu, imu.timestamp_ - last_imu_.timestamp_);
        /*
        IMU时间戳的含义：在很多IMU数据集中，时间戳标记的是采样区间的结束时刻，而不是起始时刻！

        如果时间戳是区间结束：IMU2(t=1.1) 代表 [1.0, 1.1] 的数据 → 用当前IMU正确

        如果时间戳是区间开始：IMU2(t=1.1) 代表 [1.1, 1.2] 的数据 → 用last_IMU正确



         */
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
    // tj : current_time_记录上一条记录的时间, 可能是IMU或者GNSS的时间, 它们的记录写在一起, 但是时间是升序排列, 所以这里的参照时间是上一条记录的时间
    this_gnss_ = gnss;

    if (!first_gnss_received_) {
        if (!gnss.heading_valid_) {
            // 要求首个GNSS必须有航向
            return;
        }

        // 首个gnss信号，将初始pose设置为该gnss信号
        this_frame_->timestamp_ = gnss.unix_time_;
        this_frame_->p_ = gnss.utm_pose_.translation();
        this_frame_->R_ = gnss.utm_pose_.so3();
        this_frame_->v_.setZero();
        this_frame_->bg_ = options_.preinteg_options_.init_bg_;
        this_frame_->ba_ = options_.preinteg_options_.init_ba_;

        pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_); // tj : 重置积分

        last_frame_ = this_frame_;
        last_gnss_ = this_gnss_;
        first_gnss_received_ = true;
        current_time_ = gnss.unix_time_; // tj:设置当前时间
        return;
    }

    // 积分到GNSS时刻
    pre_integ_->Integrate(last_imu_, gnss.unix_time_ - current_time_);

    current_time_ = gnss.unix_time_;
    *this_frame_ = pre_integ_->Predict(*last_frame_, options_.gravity_);

    Optimize();

    // tj : 重置后，用优化后的状态更新 last_imu_ 的时间戳
    last_imu_.timestamp_ = this_frame_->timestamp_;  // 添加这行
    // 注意：last_imu_ 的测量值（gyro/acce）无法恢复，只能保持原有值

    last_frame_ = this_frame_;
    last_gnss_ = this_gnss_;
}

void GinsPreInteg::AddOdom(const sad::Odom& odom) {
    last_odom_ = odom;
    last_odom_set_ = true;
}

void GinsPreInteg::Optimize() {
    if (pre_integ_->dt_ < 1e-3) {
        // 未得到积分
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
    // problem.SetParameterization(
    //     pose0, new ceres::ProductParameterization(new ceres::IdentityParameterization(3),
    //                                               new ceres::EigenQuaternionParameterization()));
    // problem.SetParameterization(
    //     pose1, new ceres::ProductParameterization(new ceres::IdentityParameterization(3),
    //                                               new ceres::EigenQuaternionParameterization()));


    problem.SetManifold(pose0, new ceres::ProductManifold(
        new ceres::EuclideanManifold<3>(),
        new ceres::EigenQuaternionManifold()));
    problem.SetManifold(pose1, new ceres::ProductManifold(
        new ceres::EuclideanManifold<3>(),
        new ceres::EigenQuaternionManifold()));
        
        
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
        // velocity obs
        double velo_l =
            options_.wheel_radius_ * last_odom_.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
        double velo_r =
            options_.wheel_radius_ * last_odom_.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
        double average_vel = 0.5 * (velo_l + velo_r);
        vel_odom = Vec3d(average_vel, 0.0, 0.0);

        auto* odom_cost =
            new ceres::AutoDiffCostFunction<OdomResidual, 3, 3, 7>(new OdomResidual(vel_odom, options_.odom_info_));
        problem.AddResidualBlock(odom_cost, nullptr, vel1, pose1);

        // 重置odom数据到达标志位，等待最新的odom数据
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

    // 重置integ
    options_.preinteg_options_.init_bg_ = this_frame_->bg_;  // tj : 这里重置了bg和ba
    options_.preinteg_options_.init_ba_ = this_frame_->ba_;
    pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options_); // tj : dt_ = 0; 
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