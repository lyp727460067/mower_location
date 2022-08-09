#include "neptune/location/kinamics.h"

namespace neptune {
namespace location {

OdomVelocityData
TwoWhellKinamics::ForwdVelocity(const sensor::EncoderData &encoder_data_last,
                                const sensor::EncoderData &encoder_data) {
  OdomVelocityData ret;
  ret.timestamp =
      common::ToSeconds(encoder_data.time - common::FromUniversal(0));
  double dl = encoder_data.encoder.x() - encoder_data_last.encoder.x();
  double dr = encoder_data.encoder.y() - encoder_data_last.encoder.y();
  double dt = common::ToSeconds(encoder_data.time - encoder_data_last.time);

  ret.v_b.x() = (dl + dr) / (2 * dt);
  ret.v_b.y() = 0;
  ret.v_b.z() = 0;
  ret.w_b.x() = 0;
  ret.w_b.y() = 0;
  ret.w_b.z() = (dr - dl) / (options_.b * dt);
  ret.cov.setZero();
  ImuData imu_unused;
  if (dt < 0.001) {
    ret.v_b.setZero();
    ret.w_b.setZero();
  }
  ret.cov = CovGenerate(ret, imu_unused);
  // LOG(INFO) << ret;
  return ret;
}

Eigen::Matrix<double, 6, 6> TwoWhellKinamics::CovGenerate(OdomVelocityData &cur,
                                                          ImuData &imu) {
  Eigen::Matrix<double, 6, 6> cov;
  cov.setZero();
  for (uint i = 0; i < 3; i++) {
    cov(i, i) = options_.noise_v(i);
    cov(i + 3, i + 3) = options_.noise_w(i);
  }
  return cov;
}

} // namespace location
} // namespace neptune