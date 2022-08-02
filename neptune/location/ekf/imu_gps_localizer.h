#ifndef IMU_GPS_LOCATIONLIZER
#define IMU_GPS_LOCATIONLIZER

#include <Eigen/Core>

#include "base_type.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "initializer.h"
namespace neptune {
namespace location {
struct EkfOption {
  double acc_noise;
  double gyro_noise;
  double acc_bias_noise;
  double gyro_bias_noise;
  Eigen::Vector3d imutogps_extristric;
};


class ImuGpsLocalizer {
 public:
  ImuGpsLocalizer(const EkfOption& option);
  ImuGpsLocalizer(const ImuGpsLocalizer& rhs){};
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