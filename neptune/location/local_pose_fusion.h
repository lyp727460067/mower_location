#ifndef   _LOCAL_POSE_FUSION_H
#define   _LOCAL_POSE_FUSION_H
#include <memory>
#include  "location/pose_extrapolator_interface.h"
#include "transform/transform.h"
namespace neptune {
namespace location {
struct LocalPoseFusionOption {
  double fix_weitht;
  double extraplaton_weitht;
};
class LocalPoseFusion {
 public:
  LocalPoseFusion(const LocalPoseFusionOption& option) : option_(option) {}
  std::unique_ptr<transform::Rigid3d> AddFixeData(
      const sensor::FixedFramePoseData& fix_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

 private:
  transform::Rigid3d UpdataPose(const transform::Rigid3d pose_expect,
                                const sensor::FixedFramePoseData& fix_data);
  LocalPoseFusionOption option_;
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
};

}  // namespace location
}  // namespace neptune
#endif