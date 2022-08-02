#include "imu_gps_localizer.h"
#include <glog/logging.h>
#include "utils.h"
namespace neptune {
namespace location {

ImuGpsLocalizer::ImuGpsLocalizer(const EkfOption&option)
    : initialized_(false) {

  initializer_ = std::make_unique<Initializer>(option.imutogps_extristric);
  imu_processor_ = std::make_unique<ImuProcessor>(
      option.acc_noise, option.gyro_noise, option.acc_bias_noise,
      option.gyro_bias_noise, Eigen::Vector3d(0., 0., -9.81007));
  gps_processor_ = std::make_unique<GpsProcessor>(option.imutogps_extristric);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr) {
  if (!initialized_) {
    initializer_->AddImuData(imu_data_ptr);
    return false;
  }

  // Predict.
  imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

  // Convert ENU state to lla.
  ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
  LOG(INFO)<<state_.G_p_I;
  // *fused_state = state_;
  return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(
    const GpsPositionDataPtr gps_data_ptr) {
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
}  // namespace location
}  // namespace neptune