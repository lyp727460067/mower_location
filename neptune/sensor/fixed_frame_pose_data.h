

#ifndef NEPTUNE_SENSOR_FIXED_FRAME_POSE_DATA_H_
#define NEPTUNE_SENSOR_FIXED_FRAME_POSE_DATA_H_

#include <memory>

#include "neptune/common/time.h"
#include "neptune/transform/rigid_transform.h"

namespace neptune {
namespace sensor {

// The fixed frame pose data (like GPS, pose, etc.) will be used in the
// optimization.
struct FixedFramePoseData {
  common::Time time;
  transform::Rigid3d pose;
  Eigen::Matrix3d cov;  // Covariance in m^2.
};

}  // namespace sensor
}  // namespace neptune

#endif
