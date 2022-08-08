

#ifndef LOCATION_POSE_EXTRAPOLATOR_H
#define  LOCATION_POSE_EXTRAPOLATOR_H

#include <deque>
#include <memory>
#include "neptune/location/imu_tracker.h"
#include "neptune/common/time.h"
#include "neptune/location/pose_extrapolator_interface.h"
namespace neptune {
namespace location {


class PoseExtrapolatorEkf : public PoseExtrapolatorInterface {
  public:
   explicit PoseExtrapolatorEkf(const PoseExtrapolatorEkfOption& option);

   PoseExtrapolatorEkf(const PoseExtrapolatorEkf&) = delete;
   PoseExtrapolatorEkf& operator=(const PoseExtrapolatorEkf&) = delete;

   static std::unique_ptr<PoseExtrapolatorEkf> InitializeWithImu();

   void AddPose(common::Time time, const transform::Rigid3d& pose) override;
   void AddImuData(const sensor::ImuData& imu_data) override;
   void AddOdometryData(const sensor::OdometryData& odometry_data) override;
   void AddFixedFramePoseData(
       const sensor::FixedFramePoseData& fixed_frame_pose_data) override;

   transform::Rigid3d ExtrapolatePose(common::Time time) override;

   ExtrapolationResult ExtrapolatePosesWithGravity(
       const std::vector<common::Time>& times) override;

   // Returns the current gravity alignment estimate as a rotation from
   // the tracking frame into a gravity aligned frame.
   Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

  private:
   void PredictEkfWithImu(ImuGpsLocalizer* imu_gps_location,
                          const common::Time& time);
   void PredictImu(ImuGpsLocalizer* imu_gps_location, const sensor::ImuData&imu_data);
   void UpdateVelocitiesFromPoses();
   void TrimImuData();
   void TrimOdometryData();
   void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
   Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                          ImuTracker* imu_tracker) const;
   Eigen::Vector3d ExtrapolateTranslation(common::Time time);

   //  const common::Duration pose_queue_duration_;
   struct TimedPose {
     common::Time time;
     transform::Rigid3d pose;
  };
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  std::deque<sensor::ImuData> imu_data_;
  std::deque<sensor::FixedFramePoseData> gps_data_;
  std::unique_ptr<ImuTracker> imu_tracker_;
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;
  TimedPose cached_extrapolated_pose_;
  std::deque<sensor::OdometryData> odometry_data_;

  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  std::unique_ptr<ImuGpsLocalizer> ekf_imu_gps_fustion_; 
   std::unique_ptr<ImuGpsLocalizer> ekf_imu_gps_fustion_extrapolte_; 
  State fused_state_;
};
}  // namespace location

}  // namespace neptune

#endif  // 
