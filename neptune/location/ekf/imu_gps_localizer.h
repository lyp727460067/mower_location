#ifndef IMU_GPS_LOCATIONLIZER
#define IMU_GPS_LOCATIONLIZER

#include <Eigen/Core>
#include <mutex>

#include "base_type.h"
#include "glog/logging.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "initializer.h"
#include "neptune/transform/rigid_transform.h"
#include "odom_processor.h"
namespace neptune {
namespace location {
struct EkfOption {
  double acc_noise = 1e-1;
  double gyro_noise = 1e-4;
  double acc_bias_noise = 1e-1;
  double gyro_bias_noise = 1e-8;
  transform::Rigid3d imu_to_gps;
  transform::Rigid3d imu_to_odom;
  transform::Rigid3d body_to_imu;
};

class ImuGpsLocalizer {
public:
  ImuGpsLocalizer(const EkfOption &option);
  ImuGpsLocalizer(const ImuGpsLocalizer &rhs)
      : initialized_(rhs.initialized_), init_lla_(rhs.init_lla_) {
    initializer_ = std::make_unique<Initializer>(*rhs.initializer_);
    imu_processor_ = std::make_unique<ImuProcessor>(*rhs.imu_processor_);
    gps_processor_ = std::make_unique<GpsProcessor>(*rhs.gps_processor_);
    odom_processor_ =
        std::make_unique<OdomVelocityProcessor>(*rhs.odom_processor_);
    state_ = rhs.state_;
    state_.imu_data_ptr = std::make_shared<ImuData>(*rhs.state_.imu_data_ptr);
  };
  bool ProcessImuData(const ImuDataPtr imu_data_ptr);
  const State &GetState() { return state_; }
  bool ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr);
  bool ProcessOdomData(const OdomVelocityDataPtr odom_data_ptr);
  bool Initialized() const { return initialized_; }

private:
  std::unique_ptr<Initializer> initializer_;
  std::unique_ptr<ImuProcessor> imu_processor_;
  std::unique_ptr<GpsProcessor> gps_processor_;
  std::unique_ptr<OdomVelocityProcessor> odom_processor_;

  bool initialized_;
  Eigen::Vector3d init_lla_; // The initial reference gps point.
  std::mutex state_lock_;
  State state_;
};

} // namespace location
} // namespace neptune
#endif