#include "neptune/location.h"

namespace  neptune{
namespace {
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }
Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}
}  // namespace
Eigen::Affine3d NeptuneLocation::ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude), Eigen::Vector3d::UnitZ());

  Eigen::Translation3d translatioin{rotation * -translation};
  return translatioin * rotation;
}

NeptuneLocation::NeptuneLocation(const std::string& configuration_directory,
                                 const std::string& configuration_basename) {
  options_ =
      neptune::LodeOptions(configuration_directory, configuration_basename);
  LOG(INFO) << options_.fustion_options.location_use_type;
  location_ = location::FustionInterface::CreatFusion(location::FusionOption{
      options_.fustion_options.location_use_type,
      options_.fustion_options.local_pose_option,
          location::PoseExtrapolatorEkfOption{location::EkfOption{
              options_.rigid_param.imu_instrinsci.nba.x(),
              options_.rigid_param.imu_instrinsci.nbg.x(), 0, 0,
              options_.rigid_param.sensor_extrinsic.imu_to_gps,
              options_.rigid_param.sensor_extrinsic.imu_to_odom}}});
}

void NeptuneLocation::AddFixedFramePoseData(
    const sensor::FixedFramePoseData& fix_data) {
  if (options_.fustion_options.location_use_type != 1) {
    auto const& latitude = fix_data.pose.translation().x();
    auto const& longitude = fix_data.pose.translation().y();
    auto const& altitude = fix_data.pose.translation().z();
    if (ecef_to_local_frame == nullptr) {
      ecef_to_local_frame = std::make_unique<Eigen::Affine3d>(
          ComputeLocalFrameFromLatLong(latitude, longitude));
    }
    Eigen::Vector3d lat_pose =
        *ecef_to_local_frame * LatLongAltToEcef(latitude, longitude, altitude);

    location_->AddFixedFramePoseData(
        {fix_data.time, transform::Rigid3d::Translation(lat_pose)});
    return;
  }

  location_->AddFixedFramePoseData(fix_data);
}

} // namespace neptune
