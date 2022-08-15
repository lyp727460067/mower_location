#ifndef NEPTUNE_MOTION_FILTER_H_
#define NEPTUNE_MOTION_FILTER_H_

#include <limits>

#include "neptune/common/time.h"
#include "neptune/transform/rigid_transform.h"
#include "neptune/transform/transform.h"

namespace neptune {
namespace location {
struct MotionFilterOptions {
  double max_time_seconds;
  double max_distance_meters;
  double max_angle_radians;
};

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
  const MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace location
}  // namespace neptune

#endif  
