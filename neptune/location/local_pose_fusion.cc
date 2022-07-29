#include "location/local_pose_fusion.h"

namespace neptune {
namespace location {


transform::Rigid3d UpdataPose(const transform::Rigid3d pose_expect,
                              const sensor::FixedFramePoseData& fix_data) {
 
 
                                
  }
std::unique_ptr<transform::Rigid3d> LocalPoseFusion::AddFixeData(
    const sensor::FixedFramePoseData& fix_data) {
  if (extrapolator_ == nullptr) {
    return nullptr;
  }
  transform::Rigid3d pose_expect =
      extrapolator_->ExtrapolatePose(fix_data.time);
  transform::Rigid3d pose_update = UpdataPose(pose_expect, fix_data);
  extrapolator_->AddPose(fix_data.time, pose_update);
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
