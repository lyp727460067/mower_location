

#include "neptune/sensor/imu_data.h"

#include "neptune/transform/transform.h"

namespace neptune {
namespace sensor {

// proto::ImuData ToProto(const ImuData& imu_data) {
//   proto::ImuData proto;
//   proto.set_timestamp(common::ToUniversal(imu_data.time));
//   *proto.mutable_linear_acceleration() =
//       transform::ToProto(imu_data.linear_acceleration);
//   *proto.mutable_angular_velocity() =
//       transform::ToProto(imu_data.angular_velocity);
//   return proto;
// }

// ImuData FromProto(const proto::ImuData& proto) {
//   return ImuData{common::FromUniversal(proto.timestamp()),
//                  transform::ToEigen(proto.linear_acceleration()),
//                  transform::ToEigen(proto.angular_velocity())};
// }

}  // namespace sensor
}  // namespace neptune
