#ifndef IMU_GPS_LOCATIONLIZER
#define IMU_GPS_LOCATIONLIZER

#include <Eigen/Core>

#include "base_type.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "initializer.h"
#include "glog/logging.h"
namespace neptune {
namespace location {
struct EkfOption {
  double acc_noise = 1e-2;
  double gyro_noise = 1e-4;
  double acc_bias_noise = 1e-4;
  double gyro_bias_noise = 1e-8;
  Eigen::Vector3d imutogps_extristric;
};

class ImuGpsLocalizer {
 public:
  ImuGpsLocalizer(const EkfOption& option);
  ImuGpsLocalizer(const ImuGpsLocalizer& rhs)
      : initialized_(rhs.initialized_), init_lla_(rhs.init_lla_) {
    initializer_ = std::make_unique<Initializer>(*rhs.initializer_);
    imu_processor_ = std::make_unique<ImuProcessor>(*rhs.imu_processor_);
    gps_processor_ = std::make_unique<GpsProcessor>(*rhs.gps_processor_);
    state_ = rhs.state_;
    state_.imu_data_ptr = std::make_shared<ImuData>(*rhs.state_.imu_data_ptr);
  };
  bool ProcessImuData(const ImuDataPtr imu_data_ptr);
  const State& GetState() { return state_; }
  bool ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr);

 private:
  std::unique_ptr<Initializer> initializer_;
  std::unique_ptr<ImuProcessor> imu_processor_;
  std::unique_ptr<GpsProcessor> gps_processor_;

  bool initialized_;
  Eigen::Vector3d init_lla_;  // The initial reference gps point.
  State state_;
};

}  // namespace location
}  // namespace neptune
#endif