#ifndef NEPUNE_OPTIONS_H
#define NEPUNE_OPTIONS_H
#include "transform/rigid_transform.h"
#include "Eigen/Core"
namespace neptune {
struct NeptuneOptions {
  struct RigidParm {
    struct ImuInstrinsic {
      Eigen::Vector3d ba;
      Eigen::Vector3d nba;
      Eigen::Vector3d ka;
      Eigen::Vector3d ta;
      Eigen::Vector3d bg;
      Eigen::Vector3d nbg;
      Eigen::Vector3d kg;
      Eigen::Vector3d tg;
      Eigen::Vector3d tg1;
    } imu_instrinsci;
    struct SensorExtrinsic {
      transform::Rigid3d imu_to_gps;
      transform::Rigid3d imu_to_odom;
      transform::Rigid3d bady_to_imu;
    } sensor_extrinsic;
  } rigid_param;
  struct FusionOptoin {
    bool location_use_type;
  }fustion_options;
};
NeptuneOptions LodeOptions(const std::string& configuration_directory,
                           const std::string& configuration_basename);
}  // namespace neptune

#endif