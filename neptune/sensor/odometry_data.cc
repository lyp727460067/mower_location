#include "sensor/odometry_data.h"

#include "transform/transform.h"

namespace neptune {
namespace sensor {

// proto::OdometryData ToProto(const OdometryData& odometry_data) {
//   proto::OdometryData proto;
//   proto.set_timestamp(common::ToUniversal(odometry_data.time));
//   *proto.mutable_pose() = transform::ToProto(odometry_data.pose);
//   return proto;
// }

// OdometryData FromProto(const proto::OdometryData& proto) {
//   return OdometryData{common::FromUniversal(proto.timestamp()),
//                       transform::ToRigid3(proto.pose())};
// }

}  // namespace sensor
}  // namespace cartographer
