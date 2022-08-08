#ifndef _LOCATION_H
#define  _LOCATION_H
#include "neptune/location/fusion_interface.h"
#include "neptune/neptune_options.h"
#include <string>
namespace neptune {
class NeptuneLocation {
 public:
  NeptuneLocation(const std::string& configuration_directory,
                  const std::string& configuration_basename);

  void AddFixedFramePoseData(const sensor::FixedFramePoseData& fix_data);

  void AddImuData(const sensor::ImuData& imu_data) {
    location_->AddImuData(imu_data);
  }
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid3d ExtrapolatePose(common::Time time);

 private:
  std::unique_ptr<location::FustionInterface> location_;
  NeptuneOptions options_;
};
};  // namespace neptune

#endif