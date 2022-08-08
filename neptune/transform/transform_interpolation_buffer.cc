#include "neptune/transform/transform_interpolation_buffer.h"

#include <algorithm>
#include <fstream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "neptune/transform/transform.h"
#include "glog/logging.h"
namespace neptune {
namespace transform {


TransformInterpolationBuffer::TransformInterpolationBuffer(
    const std::string& txt) {
  std::ifstream myfile;
  myfile.open(txt);
  std::string line;
  while (std::getline(myfile, line)) {
    std::istringstream iss(line);
    std::string name;
    int index;
    double x, y, z, qw, qx, qy, qz;
    int64_t tmp;
    if (!(iss >> name >> index >> x >> y >> z >> qw >> qx >> qy >> qz >> tmp)) {
      break;
    }  // error
    const common::Time time = common::FromUniversal(tmp);
    const transform::Rigid3d pose{{x, y, z}, {qw, qx, qy, qz}};
    // std::cout << "Adding time: " << time << std::endl;
    Push(time, pose);
  }
}

void TransformInterpolationBuffer::Push(const common::Time time,
                                        const transform::Rigid3d& transform) {
  if (!timestamped_transforms_.empty()) {
    CHECK_GE(time, latest_time()) << "New transform is older than latest.";
  }
  timestamped_transforms_.push_back(TimestampedTransform{time, transform});
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::SetSizeLimit(
    const size_t buffer_size_limit) {
  buffer_size_limit_ = buffer_size_limit;
  RemoveOldTransformsIfNeeded();
}

void TransformInterpolationBuffer::Clear() { timestamped_transforms_.clear(); }

bool TransformInterpolationBuffer::Has(const common::Time time) const {
  if (timestamped_transforms_.empty()) {
    LOG(INFO) << "timestamped_transforms_ empty!!!!!!!!!!!!";
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

transform::Rigid3d TransformInterpolationBuffer::Lookup(
    const common::Time time) const {
  CHECK(Has(time)) << "Missing transform for: " << time;
  const auto end = std::lower_bound(
      timestamped_transforms_.begin(), timestamped_transforms_.end(), time,
      [](const TimestampedTransform& timestamped_transform,
         const common::Time time) {
        return timestamped_transform.time < time;
      });
  if (end->time == time) {
    return end->transform;
  }
  const auto start = std::prev(end);
  return Interpolate(*start, *end, time).transform;
}

void TransformInterpolationBuffer::RemoveOldTransformsIfNeeded() {
  while (timestamped_transforms_.size() > buffer_size_limit_) {
    timestamped_transforms_.pop_front();
  }
}

common::Time TransformInterpolationBuffer::earliest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.front().time;
}

common::Time TransformInterpolationBuffer::latest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.back().time;
}

bool TransformInterpolationBuffer::empty() const {
  return timestamped_transforms_.empty();
}

size_t TransformInterpolationBuffer::size_limit() const {
  return buffer_size_limit_;
}

size_t TransformInterpolationBuffer::size() const {
  return timestamped_transforms_.size();
}
}  // namespace common
}  // namespace neptune
