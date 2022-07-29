#ifndef NEPTUTN_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define NEPTUTN_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "common/time.h"
#include "transform/rigid_transform.h"
namespace neptune {
namespace transform {

struct TimestampedTransform {
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace common
}  // namespace neptune

#endif  
