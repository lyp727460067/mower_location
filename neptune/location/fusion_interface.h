#ifndef _FUSION_INTERFACE_H
#define _FUSION_INTERFACE_H
#include <memory>

#include "neptune/location/pose_extrapolator_interface.h"
#include "neptune/transform/transform.h"

namespace neptune {
namespace location {

struct LocalPoseFusionOption {
  int fustion_type;
  double fix_weitht;
  double extraplaton_weitht;
};

struct FusionOption {
  int use_fustion_type;
  LocalPoseFusionOption local_pose_option;
  PoseExtrapolatorEkfOption ekf_option;
};

class FustionInterface {
public:
  static std::unique_ptr<FustionInterface>
  CreatFusion(const FusionOption option);
  virtual std::unique_ptr<transform::Rigid3d>
  AddFixedFramePoseData(const sensor::FixedFramePoseData &fix_data) = 0;
  virtual void AddImuData(const sensor::ImuData &imu_data) = 0;
  virtual void AddOdometryData(const sensor::OdometryData &odometry_data) = 0;
  virtual void AddEncoderData(const sensor::EncoderData &odometry_data) = 0;
  virtual transform::Rigid3d ExtrapolatePose(common::Time time) = 0;
  private:
  
};

} // namespace location
} // namespace neptune
#endif