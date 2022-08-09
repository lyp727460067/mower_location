#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <memory>
namespace neptune {
namespace location {

struct ImuData {
  double timestamp; // In second.

  Eigen::Vector3d acc;  // Acceleration in m/s^2
  Eigen::Vector3d gyro; // Angular velocity in radian/s.
  friend std::ostream &operator<<(std::ostream &os, ImuData &data) {
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << data.timestamp << " " << data.acc.transpose() << " "
       << data.gyro.transpose();
    return os;
  }
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct GpsPositionData {
  double timestamp; // In second.

  Eigen::Vector3d
      lla; // Latitude in degree, longitude in degree, and altitude in meter.
  Eigen::Matrix3d cov; // Covariance in m^2.
  friend std::ostream &operator<<(std::ostream &os, GpsPositionData &data) {
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << data.timestamp << " " << data.lla.transpose();
    return os;
  }
};

struct OdomVelocityData {
  double timestamp;
  Eigen::Vector3d v_b;
  Eigen::Vector3d w_b;
  Eigen::Matrix<double, 6, 6> cov;

  friend std::ostream &operator<<(std::ostream &os, OdomVelocityData &data) {
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << data.timestamp << " " << data.v_b.transpose() << " "
       << data.w_b.transpose();
    return os;
  }
};

using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;
using OdomVelocityDataPtr = std::shared_ptr<OdomVelocityData>;
struct State {
  double timestamp;

  Eigen::Vector3d lla; // WGS84 position.
  Eigen::Vector3d
      G_p_I; // The original point of the IMU frame in the Global frame.
  Eigen::Vector3d G_v_I; // The velocity original point of the IMU frame in the
                         // Global frame.
  Eigen::Matrix3d G_R_I; // The rotation from the IMU frame to the Global frame.
  Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
  Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.
  Eigen::Vector3d gyro;      // The gyro sensor w

  // Covariance.
  Eigen::Matrix<double, 18, 18> cov;

  // The imu data.
  ImuDataPtr imu_data_ptr;
  friend std::ostream &operator<<(std::ostream &os, State &data) {
    os << std::setprecision(std::numeric_limits<double>::max_digits10)
       << data.timestamp << " p = " << data.G_v_I.transpose()
       << " v = " << data.G_v_I.transpose()
       << " eulerAngle = " << data.G_R_I.eulerAngles(2, 1, 0).transpose()
       << " acc bias = " << data.acc_bias.transpose()
       << " gyro bias = " << data.gyro_bias.transpose()
       << " gyro = " << data.gyro.transpose() << " \ncov = \n " << data.cov;
    return os;
  }
};
} // namespace location
} // namespace neptune