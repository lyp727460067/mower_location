

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_INTERFACE_H_

#include "neptune/common/time.h"
#include "neptune/location/ekf/imu_gps_localizer.h"
#include "neptune/location/kinamics.h"
#include "neptune/sensor/encoder_data.h"
#include "neptune/sensor/fixed_frame_pose_data.h"
#include "neptune/sensor/imu_data.h"
#include "neptune/sensor/odometry_data.h"
#include "neptune/transform/transform.h"
#include <memory>
#include <tuple>
#include <vector>

namespace neptune {
namespace location {
struct PoseExtrapolatorEkfOption {
  EkfOption ekf_option;
  KinamicsOpts kinamics_option;
};

class PoseExtrapolatorInterface {
public:
  struct ExtrapolationResult {
    // The poses for the requested times at index 0 to N-1.
    std::vector<transform::Rigid3f> previous_poses;
    // The pose for the requested time at index N.
    transform::Rigid3d current_pose;
    Eigen::Vector3d current_velocity;
    Eigen::Quaterniond gravity_from_tracking;
  };

  static std::unique_ptr<PoseExtrapolatorInterface>
  CreateWithImuData(const std::vector<sensor::ImuData> &imu_data);

  PoseExtrapolatorInterface(const PoseExtrapolatorInterface &) = delete;
  PoseExtrapolatorInterface &
  operator=(const PoseExtrapolatorInterface &) = delete;
  virtual ~PoseExtrapolatorInterface() {}
  virtual void AddPose(common::Time time, const transform::Rigid3d &pose) = 0;
  virtual void AddImuData(const sensor::ImuData &imu_data) = 0;
  virtual void AddOdometryData(const sensor::OdometryData &odometry_data) = 0;
  virtual void AddEncoderData(const sensor::EncoderData &encoder_data) {
    LOG(WARNING) << "should be defined in derived class";
  }
  virtual void AddFixedFramePoseData(
      const sensor::FixedFramePoseData &fixed_frame_pose_data) = 0;
  virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;
  virtual ExtrapolationResult
  ExtrapolatePosesWithGravity(const std::vector<common::Time> &times) = 0;
  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  virtual Eigen::Quaterniond EstimateGravityOrientation(common::Time time) = 0;

protected:
  PoseExtrapolatorInterface() {}
};
} // namespace location
} // namespace neptune
#endif
