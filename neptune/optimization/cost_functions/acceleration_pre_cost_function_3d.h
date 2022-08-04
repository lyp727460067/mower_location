/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_PRE_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_PRE_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace neptune {
namespace optimization {


// Penalizes differences between IMU data and optimized accelerations.
class AccelerationPreCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor,
      const Eigen::Vector3d& delta_position_imu_frame,
      const Eigen::Vector3d& delta_velocity_imu_frame,
      const double first_delta_time_seconds) {
    return new ceres::AutoDiffCostFunction<
        AccelerationPreCostFunction3D, 6 /* residuals */,
        4 /* rotation variables */, 3 /* position variables */,
        3 /* position variables */, 3,3 /* position variables */,
        1 /* gravity variables */, 4 /* rotation variables */>(
        new AccelerationPreCostFunction3D(
            scaling_factor, delta_position_imu_frame, delta_velocity_imu_frame,
            first_delta_time_seconds));
  }

  template <typename T>
  bool operator()(const T* const start_rotation, const T* const start_position,
                  const T* const start_velocity, const T* const end_velocity,
                  const T* const end_position,
                  const T* const gravity_constant,
                  const T* const imu_calibration, T* residual) const {
    const Eigen::Quaternion<T> eigen_imu_calibration(
        imu_calibration[0], imu_calibration[1], imu_calibration[2],
        imu_calibration[3]);
    const Eigen::Matrix<T, 3, 1> imu_delta_position =
        ToEigen(start_rotation) * eigen_imu_calibration *
            delta_position_imu_frame_.cast<T>() -
        *gravity_constant *
            (0.5 * first_delta_time_seconds_ * first_delta_time_seconds_ *
             Eigen::Vector3d::UnitZ())
                .cast<T>() +
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_velocity) *
            first_delta_time_seconds_;


    //
    const Eigen::Matrix<T, 3, 1> delta_position =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_position) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_position));
    (Eigen::Map<Eigen::Matrix<T, 3, 1>>(residual) =
         T(scaling_factor_) *(imu_delta_position-delta_position));
    //
    const Eigen::Matrix<T, 3, 1> imu_delta_velocity =
        ToEigen(start_rotation) * eigen_imu_calibration *
            delta_velocity_imu_frame_.cast<T>() -
        *gravity_constant *
            ((first_delta_time_seconds_)*Eigen::Vector3d::UnitZ())
                .cast<T>();

    // LOG(INFO)<<imu_delta_velocity;
    const Eigen::Matrix<T, 3, 1> delta_velocity =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_velocity) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_velocity));
    (Eigen::Map<Eigen::Matrix<T, 3, 1>>(residual + 3) =
         T(scaling_factor_) * (imu_delta_velocity - delta_velocity));
    return true;
  }

 private:
  AccelerationPreCostFunction3D(const double scaling_factor,
                                const Eigen::Vector3d& delta_positon_imu_frame,
                                const Eigen::Vector3d& delta_velocity_imu_frame,
                                const double first_delta_time_seconds)
      : scaling_factor_(scaling_factor),
        delta_position_imu_frame_(delta_positon_imu_frame),
        delta_velocity_imu_frame_(delta_velocity_imu_frame),
        first_delta_time_seconds_(first_delta_time_seconds) {}

  AccelerationPreCostFunction3D(const AccelerationPreCostFunction3D&) = delete;
  AccelerationPreCostFunction3D& operator=(const AccelerationPreCostFunction3D&) =
      delete;

  template <typename T>
  static Eigen::Quaternion<T> ToEigen(const T* const quaternion) {
    return Eigen::Quaternion<T>(quaternion[0], quaternion[1], quaternion[2],
                                quaternion[3]);
  }

  const double scaling_factor_;
  const Eigen::Vector3d delta_position_imu_frame_;
  const Eigen::Vector3d delta_velocity_imu_frame_;
  const double first_delta_time_seconds_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
