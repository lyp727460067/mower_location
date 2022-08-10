#include "neptune/location/local_pose_fusion.h"
#include "ceres/ceres.h"
namespace neptune {
namespace location {
inline Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d &w) {
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

template <typename T> inline T NormalizeAngle(const T &angle_radians) {
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization {
public:
  template <typename T>
  bool operator()(const T *theta_radians, const T *delta_theta_radians,
                  T *theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization *Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};
class GpsCostFunction {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GpsCostFunction(const Eigen::Vector2d p_local, const Eigen::Vector2d p_gps,
                  const std::array<double, 2> &weitht)
      : p_local_(p_local), p_gps_(p_gps), weigth_(weitht) {}
  template <typename T>
  bool operator()(const T *const pose, T *residuals) const {
    Eigen::Matrix<T, 2, 1> t{pose[0], pose[1]};
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    const Eigen::Matrix<T, 2, 1> err =
        p_gps_.cast<T>() - (rotation_matrix * p_local_.cast<T>() + t);
    residuals[0] = err[0] * T(weigth_[0]);
    residuals[1] = err[1] * T(weigth_[1]);
    return true;
  }
  static ceres::CostFunction *Creat(const Eigen::Vector2d p_local,
                                    const Eigen::Vector2d p_gps,
                                    const std::array<double, 2> &weitht) {
    return new ceres::AutoDiffCostFunction<GpsCostFunction, 2, 3>(
        new GpsCostFunction(p_local, p_gps, weitht));
  }

private:
  const Eigen::Vector2d p_local_;
  const Eigen::Vector2d p_gps_;
  const std::array<double, 2> weigth_;
};

Eigen::Matrix<double, 9, 9> Fx(const Eigen::Quaterniond &q,
                               const transform::Rigid3d &delta_pose) {
  Eigen::Matrix<double, 9, 9> result = Eigen::Matrix<double, 9, 9>::Identity();
  result.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  result.block<3, 3>(3, 6) =
      -q.toRotationMatrix() * SkewSymmetric(delta_pose.translation());
  result.block<3, 3>(6, 6) = delta_pose.rotation().toRotationMatrix();
  LOG(INFO) << result;
  return result;
}

Eigen::Matrix<double, 3, 9> Hx() {
  Eigen::Matrix<double, 3, 9> result = Eigen::Matrix<double, 3, 9>::Zero();
  result.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  return result;
}

const Eigen::Matrix<double, 9, 9> PoseExtrapolatorVari() {
  Eigen::VectorXd noise = Eigen::VectorXd::Zero(9);
  noise.head<6>() = Eigen::VectorXd::Ones(6) * 0.2;
  noise.tail<3>() = Eigen::Vector3d{0.01, 0.01, 100};
  return noise.asDiagonal();
}
transform::Rigid3d LocalPoseFusion::CeresUpdata(
    const transform::Rigid3d pose_expect,
    const sensor::FixedFramePoseData& fix_data) {
  if (data_.empty()) {
    data_.push_back({pose_expect, fix_data});
    return transform::Rigid3d::Identity();
  }

  data_.push_back({pose_expect, fix_data});
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  ceres::Problem problem;
  std::array<double, 3> pose;
  for (auto data : data_) {
    problem.AddResidualBlock(
        GpsCostFunction::Creat(
            data.local_data.translation().head<2>(),
            data.fix_data.pose.translation().head<2>(),
            std::array<double, 2>{option_.fix_weitht, option_.fix_weitht}),
        nullptr, pose.data());
  }
  if (data_.size() >= 4) {
    data_.pop_front();
  }
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  transform::Rigid2d pose_gps_to_local =
      transform::Rigid2d(Eigen::Vector2d(pose[0], pose[1]), pose[2]);
  LOG(INFO)<<pose_gps_to_local;
  return transform::Embed3D(pose_gps_to_local) * pose_expect;
}
transform::Rigid3d
LocalPoseFusion::UpdataPose(const transform::Rigid3d pose_expect,
                            const sensor::FixedFramePoseData &fix_data) {
  const Eigen::Matrix3d gps_conv = Eigen::Matrix3d::Identity() * 0.1;
  const auto delta_pose = last_extrapolator_pose_.inverse() * pose_expect;
  LOG(INFO) << delta_pose;
  auto &P = conv_;
  auto const &F = Fx(pose_expect.rotation(), delta_pose);
  P = F * P * F.transpose() + PoseExtrapolatorVari();

  const auto &H = Hx();
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + gps_conv).inverse();

  LOG(INFO) << K;
  Eigen::Vector3d residual =
      (fix_data.pose.translation() - pose_expect.translation());
  LOG(INFO) << residual;
  const Eigen::VectorXd delta_x = K * residual;
  LOG(INFO) << delta_x;
  LOG(INFO) << ekf_states_;
  const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 9, 9>::Identity() - K * H;
  conv_ = I_KH * P * I_KH.transpose() + K * gps_conv * K.transpose();
  const Eigen::Vector3d detata_rotation_x = delta_x.tail(3);
  const auto &delta_rotation =
      transform::AngleAxisVectorToRotationQuaternion(detata_rotation_x);
  const double yaw = transform::GetYaw(delta_rotation);
  ekf_states_ = transform::Rigid3d(
      Eigen::Vector3d(ekf_states_.translation() + delta_x.head<3>()),
      Eigen::Quaterniond((ekf_states_.rotation() * delta_rotation))
          .normalized());
  last_extrapolator_pose_ = ekf_states_;
  return ekf_states_;
  // extrapolator_->AddPose(fix_data.time, pose_expect);
}
std::unique_ptr<transform::Rigid3d> LocalPoseFusion::AddFixedFramePoseData(
    const sensor::FixedFramePoseData &fix_data) {
  if (extrapolator_ == nullptr) {
    return nullptr;
  }
  transform::Rigid3d pose_expect =
      extrapolator_->ExtrapolatePose(fix_data.time);
  transform::Rigid3d pose_update;
  if (option_.fustion_type == 0) {
    pose_update = UpdataPose(pose_expect, fix_data);
  } else if (option_.fustion_type == 1) {
    
    pose_update = CeresUpdata(pose_expect, fix_data);
  }
  extrapolator_->AddPose(fix_data.time, pose_expect);
  ekf_states_ =  pose_update;
  return std::make_unique<transform::Rigid3d>();
}
void LocalPoseFusion::AddImuData(const sensor::ImuData &imu_data) {
  // LOG(INFO)<<imu_data.angular_velocity.transpose();
  // LOG(INFO)<<imu_data.linear_acceleration.transpose();
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  extrapolator_ =
      location::PoseExtrapolatorInterface::CreateWithImuData({imu_data});
}

transform::Rigid3d LocalPoseFusion::ExtrapolatePose(common::Time time) {
  // return transform::Rigid3d::Rotation(
  // extrapolator_->EstimateGravityOrientation(time));
  return ekf_states_;
  // return extrapolator_->ExtrapolatePose(time);
}
void LocalPoseFusion::AddOdometryData(
    const sensor::OdometryData &odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

LocalPoseFusionWithEskf::LocalPoseFusionWithEskf(
    const PoseExtrapolatorEkfOption &option)
    : option_(option) {
  extrapolator_ = std::make_unique<PoseExtrapolatorEkf>(option_);
}

void LocalPoseFusionWithEskf::AddOdometryData(
    const sensor::OdometryData &odometry_data) {
  extrapolator_->AddOdometryData(odometry_data);
}
void LocalPoseFusionWithEskf::AddImuData(const sensor::ImuData &imu_data) {
  extrapolator_->AddImuData(imu_data);
}

void LocalPoseFusionWithEskf::AddEncoderData(
    const sensor::EncoderData &encoder_data) {
  extrapolator_->AddEncoderData(encoder_data);
}

transform::Rigid3d
LocalPoseFusionWithEskf::ExtrapolatePose(const common::Time time) {
  return extrapolator_->ExtrapolatePose(time);
}

std::unique_ptr<transform::Rigid3d>
LocalPoseFusionWithEskf::AddFixedFramePoseData(
    const sensor::FixedFramePoseData &fix_data) {
  extrapolator_->AddFixedFramePoseData(fix_data);
  return std::make_unique<transform::Rigid3d>(
      extrapolator_->ExtrapolatePose(fix_data.time));
}

void PureOdomImuFusion::AddOdometryData(
    const sensor::OdometryData &odometry_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddOdometryData(odometry_data);
  }
}
void PureOdomImuFusion::AddImuData(const sensor::ImuData &imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    transform::Rigid3d pose_expect =
        extrapolator_->ExtrapolatePose(imu_data.time);
    extrapolator_->AddPose(imu_data.time, pose_expect);
    return;
  }
  extrapolator_ =
      location::PoseExtrapolatorInterface::CreateWithImuData({imu_data});
}
transform::Rigid3d PureOdomImuFusion::ExtrapolatePose(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return extrapolator_->ExtrapolatePose(time);
  }
  return {};
}

std::unique_ptr<transform::Rigid3d> PureOdomImuFusion::AddFixedFramePoseData(
    const sensor::FixedFramePoseData &fix_data) {
  return nullptr;
}

} // namespace location
} // namespace neptune
