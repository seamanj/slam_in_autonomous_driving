//
// Created by xiang on 23-1-11.
//

#include "common/g2o_types.h"

namespace sad {

EdgePriorPoseNavState::EdgePriorPoseNavState(const NavStated& state, const Mat15d& info) {
    resize(4);
    state_ = state;
    setInformation(info);
}

void EdgePriorPoseNavState::computeError() {
    auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* vv = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* vg = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* va = dynamic_cast<const VertexAccBias*>(_vertices[3]);

    // ===== 安全计算旋转误差 =====
    // 计算相对旋转矩阵
    Eigen::Matrix3d R_diff = state_.R_.matrix().transpose() * vp->estimate().so3().matrix();
    
    // 通过四元数进行"净化"，确保是正交矩阵
    Eigen::Quaterniond q_diff(R_diff);
    q_diff.normalize();
    SO3 R_diff_so3(q_diff);
    
    const Vec3d er = R_diff_so3.log();
    // =============================

    const Vec3d ep = vp->estimate().translation() - state_.p_;
    const Vec3d ev = vv->estimate() - state_.v_;
    const Vec3d ebg = vg->estimate() - state_.bg_;
    const Vec3d eba = va->estimate() - state_.ba_;

    _error << er, ep, ev, ebg, eba;
}

void EdgePriorPoseNavState::linearizeOplus() {
    const auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
    
    // ===== 安全计算旋转误差 =====
    Eigen::Matrix3d R_diff = state_.R_.matrix().transpose() * vp->estimate().so3().matrix();
    Eigen::Quaterniond q_diff(R_diff);
    q_diff.normalize();
    SO3 R_diff_so3(q_diff);
    const Vec3d er = R_diff_so3.log();
    // =============================

    /// 注意有3个index, 顶点的，自己误差的，顶点内部变量的
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = SO3::jr_inv(er);    // dr/dr    tj : 公式2.103
    _jacobianOplus[0].block<3, 3>(3, 3) = Mat3d::Identity();  // dp/dp
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(6, 0) = Mat3d::Identity();  // dv/dv
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(9, 0) = Mat3d::Identity();  // dbg/dbg
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(12, 0) = Mat3d::Identity();  // dba/dba
}

}  // namespace sad