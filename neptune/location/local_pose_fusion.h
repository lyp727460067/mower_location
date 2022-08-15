#ifndef _LOCAL_POSE_FUSION_H
#define _LOCAL_POSE_FUSION_H
#include <memory>
#include "neptune/location/motion_filter.h"
#include "neptune/location/fusion_interface.h"
#include "neptune/location/pose_extrapolator_interface.h"
#include "neptune/transform/transform.h"
namespace neptune {
namespace location {

class LocalPoseFusion : public FustionInterface {
public:
  LocalPoseFusion(const LocalPoseFusionOption &option) : option_(option) {}
  std::unique_ptr<transform::Rigid3d>
  AddFixedFramePoseData(const sensor::FixedFramePoseData &fix_data) override;
  void AddImuData(const sensor::ImuData &imu_data) override;
  void AddOdometryData(const sensor::OdometryData &odometry_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;
  void AddEncoderData(const sensor::EncoderData &encoder_data) override {}
 private:
  const bool pure_local_pose = false;
  transform::Rigid3d UpdataPose(const transform::Rigid3d pose_expect,
                                const sensor::FixedFramePoseData &fix_data);
  transform::Rigid3d CeresUpdata(const transform::Rigid3d pose_expect,
                                 const sensor::FixedFramePoseData &fix_data);
  LocalPoseFusionOption option_;
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_pub_;
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  Eigen::Matrix<double, 9, 9> conv_ =
      Eigen::Matrix<double, 9, 9>::Identity() * 100;
  std::pair<common::Time, transform::Rigid3d> pose_;
  transform::Rigid3d last_extrapolator_pose_;
  transform::Rigid3d pose_gps_to_local_;
  struct LocalData {
    transform::Rigid3d local_data;
    sensor::FixedFramePoseData fix_data;
  };

  struct CeresPose {
    std::array<double, 3> traslation;
    std::array<double, 4> rotation;
    LocalData local_data_;
  };
  // std::deque<LocalData> data_;
  std::unique_ptr<MotionFilter> motion_filter_;
  std::deque<CeresPose> slide_windows_pose_;
};
class LocalPoseFusionWithEskf : public FustionInterface {
public:
  LocalPoseFusionWithEskf(const PoseExtrapolatorEkfOption &option);
  std::unique_ptr<transform::Rigid3d>
  AddFixedFramePoseData(const sensor::FixedFramePoseData &fix_data) override;
  void AddImuData(const sensor::ImuData &imu_data) override;
  void AddOdometryData(const sensor::OdometryData &odometry_data) override;
  void AddEncoderData(const sensor::EncoderData &encoder_data) override;
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

private:
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  PoseExtrapolatorEkfOption option_;
};
class PureOdomImuFusion : public FustionInterface {
public:
  PureOdomImuFusion() {}
  std::unique_ptr<transform::Rigid3d>
  AddFixedFramePoseData(const sensor::FixedFramePoseData &fix_data) override;
  void AddImuData(const sensor::ImuData &imu_data) override;
  void AddOdometryData(const sensor::OdometryData &odometry_data) override;
  void AddEncoderData(const sensor::EncoderData &encoder_data) override {}
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

 private:
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_pub_;
  std::unique_ptr<PoseExtrapolatorInterface> extrapolator_;
  PoseExtrapolatorEkfOption option_;
};

} // namespace location
} // namespace neptune
#endif