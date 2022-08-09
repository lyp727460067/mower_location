#ifndef _UTILS_H
#define _UTILS_H
#include "base_type.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <GeographicLib/include/LocalCartesian.hpp>
#include <iomanip>

namespace neptune {
namespace location {

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

inline void ConvertLLAToENU(const Eigen::Vector3d &init_lla,
                            const Eigen::Vector3d &point_lla,
                            Eigen::Vector3d *point_enu) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                          point_enu->data()[0], point_enu->data()[1],
                          point_enu->data()[2]);
}

inline void ConvertENUToLLA(const Eigen::Vector3d &init_lla,
                            const Eigen::Vector3d &point_enu,
                            Eigen::Vector3d *point_lla) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                          point_lla->data()[0], point_lla->data()[1],
                          point_lla->data()[2]);
}

inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d w;
  w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;

  return w;
}

inline void AddDeltaToState(const Eigen::Matrix<double, 18, 1> &delta_x,
                            State *state) {
  state->G_p_I += delta_x.block<3, 1>(0, 0);
  state->G_v_I += delta_x.block<3, 1>(3, 0);
  state->acc_bias += delta_x.block<3, 1>(9, 0);
  state->gyro_bias += delta_x.block<3, 1>(12, 0);
  state->gyro += delta_x.block<3, 1>(15, 0);

  if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
    state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(),
                                      delta_x.block<3, 1>(6, 0).normalized())
                        .toRotationMatrix();
  }
}

} // namespace location
} // namespace neptune
#endif