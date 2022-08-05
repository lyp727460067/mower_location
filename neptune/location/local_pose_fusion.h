#ifndef   _LOCAL_POSE_FUSION_H
#define   _LOCAL_POSE_FUSION_H
#include <memory>

#include "location/fusion_interface.h"
#include "location/pose_extrapolator_interface.h"
#include "transform/transform.h"
#include "ekf_pose_extrapolator.h"
namespace neptune {
namespace location {

class LocalPoseFusion : public FustionInterface {
 public:
  LocalPoseFusion(const LocalPoseFusionOption& option) : option_(option) {}
  std::unique_ptr<transform::Rigid3d> AddFixedFramePoseData(
      const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

 private:
  const bool pure_local_pose = false;
  transform::Rigid3d UpdataPose(const transform::Rigid3d pose_expect,
                                const sensor::FixedFramePoseData& fix_data);
  LocalPoseFusionOption option_;
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  Eigen::Matrix<double, 6, 6> conv_ =
      Eigen::Matrix<double, 6, 6>::Identity() * 100;
  transform::Rigid3d ekf_states_;
  transform::Rigid3d last_extrapolator_pose_;
};
class LocalPoseFusionWithEskf : public FustionInterface {
 public:
  LocalPoseFusionWithEskf(const PoseExtrapolatorEkfOption& option);
  std::unique_ptr<transform::Rigid3d> AddFixedFramePoseData(
      const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

 private:
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  PoseExtrapolatorEkfOption option_;
};
class PureOdomImuFusion : public FustionInterface {
 public:
  PureOdomImuFusion() {}
  std::unique_ptr<transform::Rigid3d> AddFixedFramePoseData(
      const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

 private:
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  PoseExtrapolatorEkfOption option_;
};

}  // namespace location
}  // namespace neptune
#endif