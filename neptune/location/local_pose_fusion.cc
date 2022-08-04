#include "location/local_pose_fusion.h"

namespace neptune {
namespace location {
inline Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& w) {
  Eigen::Matrix3d w_hat;
  w_hat(0, 0) = 0;
  w_hat(0, 1) = -w(2);
  w_hat(0, 2) = w(1);
  w_hat(1, 0) = w(2);
  w_hat(1, 1) = 0;
  w_hat(1, 2) = -w(0);
  w_hat(2, 0) = -w(1);
  w_hat(2, 1) = w(0);
  w_hat(2, 2) = 0;
  return w_hat;
}

Eigen::Matrix<double, 6, 6> Fx(const Eigen::Quaterniond& q,
                               const transform::Rigid3d& delta_pose) {
  Eigen::Matrix<double, 6, 6> result = Eigen::Matrix<double, 6, 6>::Identity();
  result.block<3, 3>(3, 0) =
      -q.toRotationMatrix() * SkewSymmetric(delta_pose.translation());
  result.block<3, 3>(3, 3) = delta_pose.rotation().toRotationMatrix();
  LOG(INFO)<<result;
  return result;
}

Eigen::Matrix<double, 3, 6> Hx() {
  Eigen::Matrix<double, 3, 6> result = Eigen::Matrix<double, 3, 6>::Zero();
  result.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  return result;
}

const Eigen::Matrix<double, 6, 6> PoseExtrapolatorVari() {
  Eigen::VectorXd noise = Eigen::VectorXd::Zero(6);
  noise.head<3>() = Eigen::Vector3d::Identity() * 0.2;
  noise.tail<3>() = Eigen::Vector3d::Identity() * 10.4;
  return noise.asDiagonal();
}

transform::Rigid3d LocalPoseFusion::UpdataPose(
    const transform::Rigid3d pose_expect,
    const sensor::FixedFramePoseData& fix_data) {
  const Eigen::Matrix3d gps_conv = Eigen::Matrix3d::Identity() * 0.1;
  const auto delta_pose = last_extrapolator_pose_.inverse() * pose_expect;
  auto& P = conv_;
  auto const& F = Fx(pose_expect.rotation(), delta_pose);
  P = F * P * F.transpose() + PoseExtrapolatorVari();

  const auto& H = Hx();
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + gps_conv).inverse();

  // LOG(INFO)<<K;
  Eigen::Vector3d residual =
      (fix_data.pose.translation() - ekf_states_.translation());

  const Eigen::VectorXd delta_x = K * residual;
  const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
  conv_ = I_KH * P * I_KH.transpose() + K * gps_conv * K.transpose();
  const Eigen::Vector3d detata_rotation_x =  delta_x.head(3);
  const auto& delta_rotation =
      transform::AngleAxisVectorToRotationQuaternion(detata_rotation_x);
  ekf_states_ = transform::Rigid3d(
      Eigen::Vector3d(ekf_states_.translation() + delta_x.head<3>()),
      Eigen::Quaterniond(ekf_states_.rotation() * delta_rotation));
  last_extrapolator_pose_ = pose_expect;
  extrapolator_->AddPose(fix_data.time, ekf_states_);
}
std::unique_ptr<transform::Rigid3d> LocalPoseFusion::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fix_data) {
  if (extrapolator_ == nullptr) {
    return nullptr;
  }
  LOG(INFO)<<"add fix data";
  transform::Rigid3d pose_expect =
      extrapolator_->ExtrapolatePose(fix_data.time);
  LOG(INFO)<<pose_expect;
  // ekf_states_ =  pose_expect;
  transform::Rigid3d pose_update = UpdataPose(pose_expect, fix_data);
  extrapolator_->AddPose(fix_data.time, pose_expect);
  return std::make_unique<transform::Rigid3d>();
}
void LocalPoseFusion::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  extrapolator_ =
      location::PoseExtrapolatorInterface::CreateWithImuData({imu_data});
}

transform::Rigid3d LocalPoseFusion::ExtrapolatePose(common::Time time) {
  return transform::Rigid3d::Rotation(
      extrapolator_->EstimateGravityOrientation(time));
  // return  ekf_states_;
  // return extrapolator_->ExtrapolatePose(time);
}
void LocalPoseFusion::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}
}  // namespace location
}  // namespace neptune
