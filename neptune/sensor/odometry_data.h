#ifndef NEPTUNE_SENSOR_ODOMETRY_DATA_H_
#define NEPTUNE_SENSOR_ODOMETRY_DATA_H_

#include "neptune/common/time.h"
#include "neptune/transform/rigid_transform.h"

namespace neptune {
namespace sensor {

struct OdometryData {
  common::Time time;
  transform::Rigid3d pose;
};

// // Converts 'odometry_data' to a proto::OdometryData.
// proto::OdometryData ToProto(const OdometryData& odometry_data);

// Converts 'proto' to an OdometryData.
// OdometryData FromProto(const proto::OdometryData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ODOMETRY_DATA_H_
