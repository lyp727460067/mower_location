#include "neptune/sensor/fixed_frame_pose_data.h"

#include "neptune/transform/transform.h"

namespace neptune {
namespace sensor {

// FixedFramePoseData FromProto(const proto::FixedFramePoseData& proto) {
//   return FixedFramePoseData{common::FromUniversal(proto.timestamp()),
//                             proto.has_pose()
//                                 ? absl::optional<transform::Rigid3d>(
//                                       transform::ToRigid3(proto.pose()))
//                                 : absl::optional<transform::Rigid3d>()};
// }

}  // namespace sensor
}  // namespace neptune
