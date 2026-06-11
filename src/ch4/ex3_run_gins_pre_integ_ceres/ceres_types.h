#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "gins_pre_integ_ceres.h"

namespace sad {

// ======================== Ceres 残差块定义 ========================

// 1. 预积分残差 (15维: 位置3 + 姿态3 + 速度3 + 陀螺零偏3 + 加计零偏3)
class InertialResidual {
public:
    InertialResidual(const std::shared_ptr<IMUPreintegration>& pre_integ, const Vec3d& gravity)
        : pre_integ_(pre_integ), gravity_(gravity) {

        // 使用当前IMUPreintegration接口中的公开成员
        dp_ = pre_integ_->dp_;
        dq_ = pre_integ_->dR_;
        dv_ = pre_integ_->dv_;
        dt_ = pre_integ_->dt_;
        bg_ref_ = pre_integ_->bg_;
        ba_ref_ = pre_integ_->ba_;

        d_dbg_ = pre_integ_->dP_dbg_;
        d_dba_ = pre_integ_->dP_dba_;
        dq_dbg_ = pre_integ_->dR_dbg_;
        dv_dbg_ = pre_integ_->dV_dbg_;
        dv_dba_ = pre_integ_->dV_dba_;
    }

    template<typename T>
    bool operator()(const T* const pose0, const T* const vel0, const T* const bg0, const T* const ba0,
                    const T* const pose1, const T* const vel1, const T* const bg1, const T* const ba1,
                    T* residual) const {
        
        // 解析上一时刻状态
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p0(pose0);
        Eigen::Quaternion<T> q0(pose0[6], pose0[3], pose0[4], pose0[5]);  // w, x, y, z
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v0(vel0);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg0_(bg0);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba0_(ba0);
        
        // 解析当前时刻状态
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p1(pose1);
        Eigen::Quaternion<T> q1(pose1[6], pose1[3], pose1[4], pose1[5]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v1(vel1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg1_(bg1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba1_(ba1);
        
        // 相对预积分参考零偏的一阶修正量
        Eigen::Matrix<T, 3, 1> dbg_corr = bg0_ - bg_ref_.cast<T>();
        Eigen::Matrix<T, 3, 1> dba_corr = ba0_ - ba_ref_.cast<T>();
        
        // 对预积分测量进行一阶修正
        Eigen::Matrix<T, 3, 1> dp = dp_.cast<T>() + d_dbg_.cast<T>() * dbg_corr + d_dba_.cast<T>() * dba_corr;
        
        Eigen::Matrix<T, 3, 1> dtheta = dq_dbg_.cast<T>() * dbg_corr;
        Eigen::Quaternion<T> dq_delta(T(1), dtheta[0] / T(2), dtheta[1] / T(2), dtheta[2] / T(2));
        dq_delta.normalize();
        Eigen::Quaternion<T> dq_corr = dq_.unit_quaternion().cast<T>() * dq_delta;
        dq_corr.normalize();
        
        Eigen::Matrix<T, 3, 1> dv = dv_.cast<T>() + dv_dbg_.cast<T>() * dbg_corr + dv_dba_.cast<T>() * dba_corr;
        
        // 重力向量
        Eigen::Matrix<T, 3, 1> g = gravity_.cast<T>();
        
        // 计算残差
        Eigen::Map<Eigen::Matrix<T, 15, 1>> r(residual);
        
        // 位置残差 (3维)
        r.template block<3, 1>(0, 0) = q0.conjugate() * (p1 - p0 - v0 * dt_ - T(0.5) * g * dt_ * dt_) - dp;
        
        // 姿态残差 (3维，使用对数映射)
        Eigen::Quaternion<T> q_error = dq_corr.conjugate() * q0.conjugate() * q1;
        T q_error_wxyz[4] = {q_error.w(), q_error.x(), q_error.y(), q_error.z()};
        ceres::QuaternionToAngleAxis(q_error_wxyz, r.template block<3, 1>(3, 0).data());
        
        // 速度残差 (3维)
        r.template block<3, 1>(6, 0) = q0.conjugate() * (v1 - v0 - g * dt_) - dv;
        
        // 陀螺零偏残差 (3维)
        r.template block<3, 1>(9, 0) = bg1_ - bg0_;
        
        // 加计零偏残差 (3维)
        r.template block<3, 1>(12, 0) = ba1_ - ba0_;
        
        return true;
    }

private:
    std::shared_ptr<IMUPreintegration> pre_integ_;
    Vec3d dp_, dv_, bg_ref_, ba_ref_, gravity_;
    SO3 dq_;
    double dt_;
    Mat3d d_dbg_, d_dba_, dq_dbg_, dv_dbg_, dv_dba_;
};

// 2. 陀螺仪零偏随机游走残差
class GyroRWResidual {
public:
    GyroRWResidual(const Mat3d& info) {
        sqrt_info_ = info.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const bg0, const T* const bg1, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg0_(bg0);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg1_(bg1);
        
        Eigen::Matrix<T, 3, 1> error = bg1_ - bg0_;
        
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residual);
        res = sqrt_info_.cast<T>() * error;
        return true;
    }

private:
    Mat3d sqrt_info_;
};

// 3. 加速度计零偏随机游走残差
class AccRWResidual {
public:
    AccRWResidual(const Mat3d& info) {
        sqrt_info_ = info.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const ba0, const T* const ba1, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba0_(ba0);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba1_(ba1);
        
        Eigen::Matrix<T, 3, 1> error = ba1_ - ba0_;
        
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residual);
        res = sqrt_info_.cast<T>() * error;
        return true;
    }

private:
    Mat3d sqrt_info_;
};

// 4. GNSS残差 (6维: 姿态3 + 位置3)
class GNSSResidual {
public:
    GNSSResidual(const SE3& utm_pose, const Mat6d& info) 
        : p_gnss_(utm_pose.translation()), 
          q_gnss_(utm_pose.unit_quaternion()) {
        sqrt_info_ = info.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const pose, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(pose);
        Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
        
        // 位置误差
        Eigen::Matrix<T, 3, 1> pos_error = p - p_gnss_.cast<T>();
        
        // 姿态误差
        Eigen::Quaternion<T> q_error = q_gnss_.cast<T>().conjugate() * q;
        Eigen::Matrix<T, 3, 1> rot_error;
        T q_error_wxyz[4] = {q_error.w(), q_error.x(), q_error.y(), q_error.z()};
        ceres::QuaternionToAngleAxis(q_error_wxyz, rot_error.data());
        
        // 组合残差
        Eigen::Matrix<T, 6, 1> err;
        err.template block<3, 1>(0, 0) = rot_error;
        err.template block<3, 1>(3, 0) = pos_error;
        
        Eigen::Map<Eigen::Matrix<T, 6, 1>> res(residual);
        res = sqrt_info_.cast<T>() * err;
        
        return true;
    }

private:
    Vec3d p_gnss_;
    Eigen::Quaterniond q_gnss_;
    Mat6d sqrt_info_;
};

// 5. 先验残差 (15维最小形式: 位置3 + 姿态3 + 速度3 + 陀螺零偏3 + 加计零偏3)
class PriorResidual {
public:
    PriorResidual(const NavStated& state, const Mat15d& info)
        : p_prior_(state.p_),
          q_prior_(state.R_.unit_quaternion()),
          v_prior_(state.v_),
          bg_prior_(state.bg_),
          ba_prior_(state.ba_) {
        sqrt_info_ = info.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const pose, const T* const vel, const T* const bg, const T* const ba, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(pose);
        Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v(vel);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg_(bg);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_(ba);

        Eigen::Matrix<T, 15, 1> error;
        error.template block<3, 1>(0, 0) = p - p_prior_.cast<T>();

        Eigen::Quaternion<T> q_error = q_prior_.cast<T>().conjugate() * q;
        T q_error_wxyz[4] = {q_error.w(), q_error.x(), q_error.y(), q_error.z()};
        ceres::QuaternionToAngleAxis(q_error_wxyz, error.template block<3, 1>(3, 0).data());

        error.template block<3, 1>(6, 0) = v - v_prior_.cast<T>();
        error.template block<3, 1>(9, 0) = bg_ - bg_prior_.cast<T>();
        error.template block<3, 1>(12, 0) = ba_ - ba_prior_.cast<T>();

        Eigen::Map<Eigen::Matrix<T, 15, 1>> res(residual);
        res = sqrt_info_.cast<T>() * error;

        return true;
    }

private:
    Vec3d p_prior_, v_prior_, bg_prior_, ba_prior_;
    Eigen::Quaterniond q_prior_;
    Mat15d sqrt_info_;
};

// 6. 里程计残差 (3维速度)
class OdomResidual {
public:
    OdomResidual(const Vec3d& vel_odom, const Mat3d& info) : vel_odom_(vel_odom) {
        sqrt_info_ = info.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const vel, const T* const pose, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> v(vel);
        
        // 将速度转到body系下进行比较
        Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Matrix<T, 3, 1> v_body = q.conjugate() * v;
        
        Eigen::Matrix<T, 3, 1> error = v_body - vel_odom_.cast<T>();
        
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residual);
        res = sqrt_info_.cast<T>() * error;
        
        return true;
    }

private:
    Vec3d vel_odom_;
    Mat3d sqrt_info_;
};

}  // namespace sad