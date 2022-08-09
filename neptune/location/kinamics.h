#ifndef KINAMICS_H
#define KINAMICS_H
#include "neptune/location/ekf/imu_gps_localizer.h"
#include "neptune/sensor/encoder_data.h"
#include <Eigen/Core>
namespace neptune {
namespace location {

struct KinamicsOpts {
  double b = 0.3;   // whell distance unit: m
  double r = 0.075; // whell radius
  Eigen::Vector3d noise_v = {0.1, 0.08, 0.08};
  Eigen::Vector3d noise_w = {0.1, 0.1, 0.05};
};

class KinamicsInterface {
public:
  KinamicsInterface() {}
  virtual OdomVelocityData
  ForwdVelocity(const sensor::EncoderData &encoder_data_last,
                const sensor::EncoderData &encoder_data) = 0;
  virtual void ForwdPose(){};
};

class TwoWhellKinamics : public KinamicsInterface {
public:
  TwoWhellKinamics(const KinamicsOpts &opt) : options_(opt) {}
  OdomVelocityData
  ForwdVelocity(const sensor::EncoderData &encoder_data_last,
                const sensor::EncoderData &encoder_data) override;
  Eigen::Matrix<double, 6, 6> CovGenerate(OdomVelocityData &cur, ImuData &imu);

private:
  const KinamicsOpts options_;
};

} // namespace location
} // namespace neptune

#endif