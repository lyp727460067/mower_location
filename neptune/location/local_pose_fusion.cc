#include "neptune/location/local_pose_fusion.h"

#include "ceres/ceres.h"
#include "neptune/location/pose_extrapolator.h"
#include <Eigen/Core>
#include "neptune/location/ekf_pose_extrapolator.h"
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

template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
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

template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
  const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                         -start_rotation[2],
                                         -start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                     end_translation[1] - start_translation[1],
                                     end_translation[2] - start_translation[2]);
  const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

  const Eigen::Quaternion<T> h_rotation_inverse =
      Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                           -end_rotation[3]) *
      Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                           start_rotation[2], start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> angle_axis_difference =
      transform::RotationQuaternionToAngleAxisVector(
          h_rotation_inverse * relative_pose.rotation().cast<T>());

  return {{T(relative_pose.translation().x()) - h_translation[0],
           T(relative_pose.translation().y()) - h_translation[1],
           T(relative_pose.translation().z()) - h_translation[2],
           angle_axis_difference[0], angle_axis_difference[1],
           angle_axis_difference[2]}};
}
template <typename T>
std::array<T, 6> ScaleError(const std::array<T, 6>& error,
                            double translation_weight, double rotation_weight) {
  // clang-format off
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * translation_weight,
      error[3] * rotation_weight,
      error[4] * rotation_weight,
      error[5] * rotation_weight
  }};
  // clang-format on
}


class GpsWindCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GpsWindCostFunction(const transform::Rigid3d& pose,
                      const std::array<double, 2>& weitht)
      : mearemnt_pose_(pose), weigth_(weitht) {}
  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(mearemnt_pose_, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        weigth_[0], weigth_[1]);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const transform::Rigid3d& pose, const std::array<double, 2>& weitht) {
    return new ceres::AutoDiffCostFunction<GpsWindCostFunction, 6, 4, 3, 4, 3>(
        new GpsWindCostFunction(pose, weitht));
  }

 private:
  const transform::Rigid3d mearemnt_pose_;
  const std::array<double, 2> weigth_;
};
class Gps2dWindCostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Gps2dWindCostFunction(const transform::Rigid3d& pose,
                      const std::array<double, 2>& weitht)
      : mearemnt_pose_(pose), weigth_(weitht) {}
  template <typename T>
  bool operator()(const T* const c_i_angle, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    std::array<T, 4> c_i_rotation{
        {cos(c_i_angle[0] * T(0.5)), T(0), T(0), sin(c_i_angle[0] *T(0.5))}};
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(mearemnt_pose_, c_i_rotation.data(), c_i_translation,
                             c_j_rotation, c_j_translation),
        weigth_[0], weigth_[1]);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const transform::Rigid3d& pose, const std::array<double, 2>& weitht) {
    return new ceres::AutoDiffCostFunction<Gps2dWindCostFunction, 6, 1, 3, 4, 3>(
        new Gps2dWindCostFunction(pose, weitht));
  }

 private:
  const transform::Rigid3d mearemnt_pose_;
  const std::array<double, 2> weigth_;
};


Eigen::Matrix<double, 9, 9> Fx(const Eigen::Quaterniond& q,
                               const transform::Rigid3d& delta_pose) {
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
LocalPoseFusion::LocalPoseFusion(const LocalPoseFusionOption& option)
    : option_(option) {
  motion_filter_ =
      std::make_unique<MotionFilter>(MotionFilterOptions{100, 0.1, 3.1415});
}
std::pair<std::array<double, 3>, std::array<double, 4>> ToCeresPose(
    const transform::Rigid3d& pose) {
  return {{
              {pose.translation().x(), pose.translation().y(),
               pose.translation().z()},
          },
          {
              {pose.rotation().w(), pose.rotation().x(), pose.rotation().y(),
               pose.rotation().z()},

          }};
}
transform::Rigid3d ArrayToRigid(
    const std::pair<std::array<double, 3>, std::array<double, 4>>& pose) {
  return transform::Rigid3d(
      Eigen::Vector3d(pose.first[0], pose.first[1], pose.first[2]),
      Eigen::Quaterniond(pose.second[0], pose.second[1], pose.second[2],
                         pose.second[3]));
}
transform::Rigid3d LocalPoseFusion::CeresUpdata(
    const transform::Rigid3d pose_expect,
    const sensor::FixedFramePoseData& fix_data) {
  const auto ceres_pose = ToCeresPose(pose_expect);
  if (slide_windows_pose_.size() < 2) {
    slide_windows_pose_.push_back(
        {ceres_pose.first, ceres_pose.second, {pose_expect, fix_data}});
    return transform::Rigid3d::Identity();
  }
  slide_windows_pose_.push_back({slide_windows_pose_.back().traslation,
                                 slide_windows_pose_.back().rotation,
                                 {pose_expect, fix_data}});
  ceres::LocalParameterization* quaternion_local =
      new ceres::EigenQuaternionParameterization;
  ceres::Problem problem;
  auto fix_local_to_map_ = transform::Rigid3d(
      pose_gps_to_local_.translation(),
      Eigen::AngleAxisd(transform::GetYaw(pose_gps_to_local_.rotation()),
                        Eigen::Vector3d::UnitZ()));
  auto fix_local_to_map_arry = ToCeresPose(fix_local_to_map_);
  double fix_local_to_map_arry_angle =
      transform::GetYaw(pose_gps_to_local_.rotation());
  for (auto& slide_widons_data : slide_windows_pose_) {
    problem.AddResidualBlock(
        Gps2dWindCostFunction::CreateAutoDiffCostFunction(
            slide_widons_data.local_data_.fix_data.pose,
            std::array<double, 2>{option_.fix_weitht_traslation,
                                  option_.fix_weitht_rotation}),
        new ceres::HuberLoss(0.01), &fix_local_to_map_arry_angle,
        fix_local_to_map_arry.first.data(), slide_widons_data.rotation.data(),
        slide_widons_data.traslation.data());
    // problem.SetParameterization(fix_local_to_map_arry.second.data(),
    //                             quaternion_local);
    problem.SetParameterization(slide_widons_data.rotation.data(),
                                quaternion_local);
  }

  // problem.SetParameterUpperBound(fix_local_to_map_arry.second.data(), 1, 0.0);
  // problem.SetParameterUpperBound(fix_local_to_map_arry.second.data(), 2, 0.0);
  std::array<double, 3> traslation{0, 0, 0};
  std::array<double, 4> rotation{1, 0, 0, 0};
  for (int i = 0; i < slide_windows_pose_.size(); i++) {
    problem.AddResidualBlock(
        GpsWindCostFunction::CreateAutoDiffCostFunction(
            slide_windows_pose_[i].local_data_.local_data,
            {option_.extraplaton_weitht, option_.extraplaton_weitht}),
        nullptr, rotation.data(), traslation.data(),
        slide_windows_pose_[i].rotation.data(),
        slide_windows_pose_[i].traslation.data());
  }
  problem.SetParameterBlockConstant(traslation.data());
  problem.SetParameterBlockConstant(rotation.data());
  // for (int i = 0; i < slide_windows_pose_.size(); i++) {
  //   problem.AddResidualBlock(
  //       GpsWindCostFunction::CreateAutoDiffCostFunction(
  //           slide_windows_pose_[i - 1].local_data_.local_data.inverse() *
  //               slide_windows_pose_[i].local_data_.local_data,
  //           {option_.extraplaton_weitht, option_.extraplaton_weitht}),
  //       nullptr, slide_windows_pose_[i - 1].rotation.data(),
  //       slide_windows_pose_[i - 1].traslation.data(),
  //       slide_windows_pose_[i ].rotation.data(),
  //       slide_windows_pose_[i ].traslation.data());
  // }
  
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  auto pose_gps_to_local_temp = ArrayToRigid(fix_local_to_map_arry);
  pose_gps_to_local_ =
      transform::Rigid3d(pose_gps_to_local_temp.translation(),
                         Eigen::AngleAxisd((fix_local_to_map_arry_angle),
                                           Eigen::Vector3d::UnitZ()));
  auto const pose_optimazation =
      ArrayToRigid({slide_windows_pose_.back().traslation,
                    slide_windows_pose_.back().rotation});
  if (slide_windows_pose_.size() >= option_.slide_windows_num) {
    slide_windows_pose_.pop_front();
  }
  LOG(INFO)<<pose_gps_to_local_;
  LOG(INFO)<<pose_optimazation;
  return  pose_optimazation;
}
transform::Rigid3d
LocalPoseFusion::UpdataPose(const transform::Rigid3d pose_expect,
                            const sensor::FixedFramePoseData &fix_data) {
  const Eigen::Matrix3d gps_conv = Eigen::Matrix3d::Identity() * 0.1;
  const auto delta_pose = last_extrapolator_pose_.inverse() * pose_expect;
  auto& P = conv_;
  auto const& F = Fx(pose_expect.rotation(), delta_pose);
  P = F * P * F.transpose() + PoseExtrapolatorVari();

  const auto &H = Hx();
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + gps_conv).inverse();

  Eigen::Vector3d residual =
      (fix_data.pose.translation() - pose_expect.translation());
  const Eigen::VectorXd delta_x = K * residual;
  const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 9, 9>::Identity() - K * H;
  conv_ = I_KH * P * I_KH.transpose() + K * gps_conv * K.transpose();
  const Eigen::Vector3d detata_rotation_x = delta_x.tail(3);
  const auto &delta_rotation =
      transform::AngleAxisVectorToRotationQuaternion(detata_rotation_x);
  const double yaw = transform::GetYaw(delta_rotation);
  pose_ = {fix_data.time,
           transform::Rigid3d(
               Eigen::Vector3d(pose_.second.translation() + delta_x.head<3>()),
               Eigen::Quaterniond((pose_.second.rotation() * delta_rotation))
                   .normalized())};
  last_extrapolator_pose_ = pose_expect;
  return pose_.second;
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
    if (motion_filter_->IsSimilar(fix_data.time, pose_expect)) {
      pose_update = pose_expect;
    } else {
      pose_update = CeresUpdata(pose_expect, fix_data);
      if (extrapolator_pub_ == nullptr) {
        extrapolator_pub_ =
            std::make_unique<PoseExtrapolator>(common::FromSeconds(0.001), 10.);
      }
      extrapolator_pub_->AddPose(fix_data.time,
                                 pose_gps_to_local_.inverse() * pose_update);
    }
  }

  extrapolator_->AddPose(fix_data.time,pose_expect);
  return std::make_unique<transform::Rigid3d>();
}
void LocalPoseFusion::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    if (extrapolator_pub_ != nullptr) {
      extrapolator_pub_->AddImuData(imu_data);
    }
    return;
  }
  extrapolator_ =
      location::PoseExtrapolatorInterface::CreateWithImuData({imu_data});
}
transform::Rigid3d LocalPoseFusion::ExtrapolatePose(common::Time time) {
  if (extrapolator_pub_ != nullptr) {
   return  extrapolator_pub_->ExtrapolatePose(time);
  }
  return transform::Rigid3d::Identity();
}
void LocalPoseFusion::AddOdometryData(
    const sensor::OdometryData &odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
  if (extrapolator_pub_ != nullptr) {
    extrapolator_pub_->AddOdometryData(odometry_data);
  }
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
