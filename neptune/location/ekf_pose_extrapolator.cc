#include "location/ekf_pose_extrapolator.h"

#include <algorithm>

#include "glog/logging.h"
#include "transform/transform.h"
namespace neptune {
namespace location {
PoseExtrapolatorEkf::PoseExtrapolatorEkf(
    const PoseExtrapolatorEkfOption& option)
    : cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {
  ekf_imu_gps_fustion_ = std::make_unique<ImuGpsLocalizer>(option.ekf_option);
};
std::unique_ptr<PoseExtrapolatorEkf> PoseExtrapolatorEkf::InitializeWithImu() {

}
void PoseExtrapolatorEkf::AddPose(const common::Time time,
                                  const transform::Rigid3d& pose) {
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - common::FromSeconds(0.001)) {
    timed_pose_queue_.pop_front();
  }
  TrimImuData();
  TrimOdometryData();
}

void PoseExtrapolatorEkf::AddImuData(const sensor::ImuData& imu_data) {
  // if (timed_pose_queue_.empty()) {
  //   return;
  // }
  // if (imu_data.time < timed_pose_queue_.back().time) {
  //   return;
  // }
  imu_data_.push_back(imu_data);
  // TrimImuData();
}

void PoseExtrapolatorEkf::AddOdometryData(
    const sensor::OdometryData& odometry_data) {}


Eigen::Quaterniond PoseExtrapolatorEkf::EstimateGravityOrientation(
    const common::Time time) {}

void PoseExtrapolatorEkf::UpdateVelocitiesFromPoses() {}

void PoseExtrapolatorEkf::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolatorEkf::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

void PoseExtrapolatorEkf::PredictImu(ImuGpsLocalizer* imu_gps_location,
                                     const sensor::ImuData& imu_data) {
  imu_gps_location->ProcessImuData(std::make_shared<ImuData>(
      ImuData{common::ToSeconds(imu_data.time - common::FromUniversal(0)),
              imu_data.linear_acceleration, imu_data.angular_velocity}));
}

void PoseExtrapolatorEkf::PredictEkfWithImu(ImuGpsLocalizer* imu_gps_location,
                                            const common::Time& time) {
  auto it = imu_data_.begin();
  while (it != imu_data_.end() && it->time < time) {
    PredictImu(imu_gps_location, *it);
    ++it;
  }
}

void PoseExtrapolatorEkf::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  const auto& time = fixed_frame_pose_data.time;
 
  PredictEkfWithImu(ekf_imu_gps_fustion_.get(), time);
  const auto& fix_data = fixed_frame_pose_data;
  if (!ekf_imu_gps_fustion_->ProcessGpsPositionData(
          std::make_shared<GpsPositionData>(GpsPositionData{
              common::ToSeconds(fix_data.time - common::FromUniversal(0)),
              fix_data.pose.translation(), fix_data.cov}))) {
    AddPose(fix_data.time, transform::Rigid3d::Identity());
    return;
  }
  LOG(INFO)<<"updata";
  ekf_imu_gps_fustion_extrapolte_ =
      std::make_unique<ImuGpsLocalizer>(*ekf_imu_gps_fustion_);
  AddPose(fix_data.time,
          transform::Rigid3d{fused_state_.G_p_I,
                             Eigen::Quaterniond(fused_state_.G_R_I)});
}

transform::Rigid3d PoseExtrapolatorEkf::ExtrapolatePose(
    const common::Time time) {
  if (timed_pose_queue_.empty()) return transform::Rigid3d::Identity();
  if (ekf_imu_gps_fustion_extrapolte_ == nullptr)
    return transform::Rigid3d::Identity();
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    PredictEkfWithImu(ekf_imu_gps_fustion_extrapolte_.get(), time);
    auto state = ekf_imu_gps_fustion_extrapolte_->GetState();
    cached_extrapolated_pose_ = TimedPose{
        time, transform::Rigid3d{state.G_p_I, Eigen::Quaterniond(state.G_R_I)}};
  }
  return cached_extrapolated_pose_.pose;
}

void PoseExtrapolatorEkf::AdvanceImuTracker(
    const common::Time time, ImuTracker* const imu_tracker) const {}

Eigen::Quaterniond PoseExtrapolatorEkf::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {}

Eigen::Vector3d PoseExtrapolatorEkf::ExtrapolateTranslation(common::Time time) {

}

PoseExtrapolatorEkf::ExtrapolationResult
PoseExtrapolatorEkf::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {}
}  // namespace location
}  // namespace neptune
