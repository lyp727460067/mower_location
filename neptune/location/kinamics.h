#ifndef KINAMICS_H
#define KINAMICS_H
#include "neptune/location/ekf/imu_gps_localizer.h"
#include "neptune/sensor/encoder_data.h"
#include <Eigen/Core>
#include <iostream>
namespace neptune {
namespace location {

struct KinamicsOpts {
  double b = 0.3;   // whell distance unit: m
  double r = 0.075; // whell radius
  Eigen::Vector3d noise_v = {0.01, 0.01, 0.01};
  Eigen::Vector3d noise_w = {0.01, 0.01, 0.01};
  friend std::ostream &operator<<(std::ostream &os, const KinamicsOpts &data) {
    os << " b = " << data.b << " r = " << data.r
       << " noise_v = " << data.noise_v.transpose()
       << " noise_w = " << data.noise_w.transpose();
    return os;
  }
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
  TwoWhellKinamics(const KinamicsOpts &opt) : options_(opt) {
    LOG(INFO) << "construct TwoWhellKinamics with " << options_;
  }
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