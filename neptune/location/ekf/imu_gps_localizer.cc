#include "imu_gps_localizer.h"
#include "utils.h"
#include <glog/logging.h>
namespace neptune {
namespace location {

ImuGpsLocalizer::ImuGpsLocalizer(const EkfOption &option)
    : initialized_(false) {
  initializer_ =
      std::make_unique<Initializer>(option.imu_to_gps.inverse().translation());
  imu_processor_ = std::make_unique<ImuProcessor>(
      option.acc_noise, option.gyro_noise, option.acc_bias_noise,
      option.gyro_bias_noise, Eigen::Vector3d(0., 0., -9.81007));
  gps_processor_ =
      std::make_unique<GpsProcessor>(option.imu_to_gps.inverse().translation());
  odom_processor_ = std::make_unique<OdomVelocityProcessor>(
      option.imu_to_odom.inverse().translation(),
      option.imu_to_odom.inverse().rotation().toRotationMatrix());
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr) {
  std::lock_guard<std::mutex> lck(state_lock_);
  if (!initialized_) {
    initializer_->AddImuData(imu_data_ptr);
    return false;
  }

  // LOG(INFO) << "process imu data "
  //           << std::setprecision(std::numeric_limits<double>::max_digits10)
  //           << imu_data_ptr->timestamp;
  // Predict.
  imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

  // Convert ENU state to lla.
  ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
  // *fused_state = state_;
  return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(
    const GpsPositionDataPtr gps_data_ptr) {
  std::lock_guard<std::mutex> lck(state_lock_);
  // LOG(INFO) << "process gps "
  //           << std::setprecision(std::numeric_limits<double>::max_digits10)
  //           << gps_data_ptr->timestamp << gps_data_ptr->lla.transpose();

  if (!initialized_) {
    if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
      return false;
    }

    // Initialize the initial gps point used to convert lla to ENU.
    init_lla_ = gps_data_ptr->lla;

    initialized_ = true;

    LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
    return true;
  }

  // Update.
  gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);

  return true;
}

bool ImuGpsLocalizer::ProcessOdomData(const OdomVelocityDataPtr odom_data_ptr) {
  std::lock_guard<std::mutex> lck(state_lock_);

  if (!initialized_) {
    LOG(WARNING) << "not inited";
    return false;
  }
  // Update.
  // LOG(INFO) << "process Odom data "
  //           << std::setprecision(std::numeric_limits<double>::max_digits10)
  //           << odom_data_ptr->timestamp;

  odom_processor_->UpdateStateByOdomVelocity(odom_data_ptr, &state_);

  return true;
}

} // namespace location
} // namespace neptune