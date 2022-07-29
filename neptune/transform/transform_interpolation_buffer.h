

#ifndef NEPTUTN_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define NEPTUTN_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <limits>

#include "common/time.h"
#include "transform/timestamped_transform.h"

namespace neptune {
namespace transform {
  
constexpr size_t kUnlimitedBufferSize = std::numeric_limits<size_t>::max();

// A time-ordered buffer of transforms that supports interpolated lookups.
// Unless explicitly set, the buffer size is unlimited.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  TransformInterpolationBuffer(const std::string& txt);
  // Sets the transform buffer size limit and removes old transforms
  // if it is exceeded.
  void SetSizeLimit(size_t buffer_size_limit);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Clears the transform buffer.
  void Clear();

  // Returns true if an interpolated transform can be computed at 'time'.
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

  // Returns the maximum allowed size of the transform buffer.
  size_t size_limit() const;

  // Returns the current size of the transform buffer.
  size_t size() const;

 private:
  void RemoveOldTransformsIfNeeded();

  std::deque<TimestampedTransform> timestamped_transforms_;
  size_t buffer_size_limit_ = kUnlimitedBufferSize;
};
}  // namespace common
}  // namespace neptune

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
