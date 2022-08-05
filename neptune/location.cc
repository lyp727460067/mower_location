#include "location.h"
namespace  neptune{
namespace {


}  // namespace

NeptuneLocation::NeptuneLocation(const std::string& configuration_directory,
                                 const std::string& configuration_basename) {
  options_ =
      neptune::LodeOptions(configuration_directory, configuration_basename);
  location_ = location::FustionInterface::CreatFusion(location::FusionOption{
      options_.fustion_options.location_use_type,
      location::LocalPoseFusionOption{0, 0},
      location::PoseExtrapolatorEkfOption{location::EkfOption{
          options_.rigid_param.imu_instrinsci.nba.x(),
          options_.rigid_param.imu_instrinsci.nbg.x(), 0, 0,
          options_.rigid_param.sensor_extrinsic.imu_to_gps,
          options_.rigid_param.sensor_extrinsic.imu_to_odom}}});
}

void NeptuneLocation::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fix_data) {
  location_->AddFixedFramePoseData(fix_data);
}

}  // namespace neptune
