#ifndef NEPTUNE_SENSOR_IMU_DATA_H_
#define NEPTUNE_SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "common/time.h"

namespace neptune {
namespace sensor {

struct ImuData {
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

}  // namespace sensor
}  // namespace neptune

#endif  // CARTOGRAPHER_SENSOR_IMU_DATA_H_
