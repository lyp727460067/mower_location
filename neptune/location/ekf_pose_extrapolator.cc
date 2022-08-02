#include <algorithm>
#include "glog/logging.h"
#include "transform/transform.h"
#include "location/ekf_pose_extrapolator.h"
namespace neptune {
namespace location {
PoseExtrapolatorEkf::PoseExtrapolatorEkf(
    const PoseExtrapolatorEkfOption& option)
    : cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {
  ekf_imu_gps_fustion_ = std::make_unique<ImuGpsLocalizer>(option.ekf_option);
};
std::unique_ptr<PoseExtrapolatorEkf> PoseExtrapolatorEkf::InitializeWithImu() {}
void PoseExtrapolatorEkf::AddPose(const common::Time time,
                                  const transform::Rigid3d& pose) {}

void PoseExtrapolatorEkf::AddImuData(const sensor::ImuData& imu_data) {
  if (timed_pose_queue_.empty()) {
    return;
  }
  if (imu_data.time < timed_pose_queue_.back().time) {
    return;
  }
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolatorEkf::AddOdometryData(
    const sensor::OdometryData& odometry_data) {}

transform::Rigid3d PoseExtrapolatorEkf::ExtrapolatePose(
    const common::Time time) {}
Eigen::Quaterniond PoseExtrapolatorEkf::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

void PoseExtrapolatorEkf::UpdateVelocitiesFromPoses() {
}

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

void PoseExtrapolatorEkf::PredictEkfWithImu(const common::Time& time) {
  auto it = imu_data_.begin();
  while (it != imu_data_.end() && it->time < time) {
    ekf_imu_gps_fustion_->ProcessImuData({
        common::ToSeconds(time-common::Time(0)),
    });
  }
}

void PoseExtrapolatorEkf::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  const auto &time = fixed_frame_pose_data.time;

  // timed_pose_queue_.push_back(TimedPose{time, pose});
  // while (timed_pose_queue_.size() > 2 &&
  //        timed_pose_queue_[1].time <= time - pose_queue_duration_) {
  //   timed_pose_queue_.pop_front();
  // }
  PredictEkfWithImu();

  // UpdateVelocitiesFromPoses();
  // AdvanceImuTracker(time, imu_tracker_.get());
  // TrimImuData();
  // TrimOdometryData();

}

void PoseExtrapolatorEkf::AdvanceImuTracker(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolatorEkf::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolatorEkf::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

PoseExtrapolatorEkf::ExtrapolationResult
PoseExtrapolatorEkf::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}
}  // namespace location
}  // namespace neptune
