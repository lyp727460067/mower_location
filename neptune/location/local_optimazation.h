#ifndef _LOCAL_OPTIMAZATION_H
#define  _LOCAL_OPTIMAZATION_H

template <typename T>
struct IntegrateImuResult {
  Eigen::Matrix<T, 3, 1> delta_velocity;
  Eigen::Matrix<T, 3, 1> delta_translation;
  Eigen::Quaternion<T> delta_rotation;
};

template <typename T, typename RangeType, typename IteratorType>
IntegrateImuResult<T> IntegrateImu(
    const RangeType& imu_data,
    const Eigen::Transform<T, 3, Eigen::Affine>&
        linear_acceleration_calibration,
    const Eigen::Transform<T, 3, Eigen::Affine>& angular_velocity_calibration,
    const common::Time start_time, const common::Time end_time,
    IteratorType* const it) {
  CHECK_LE(start_time, end_time);
  CHECK(*it != imu_data.end());
  CHECK_LE((*it)->time, start_time);
  if (std::next(*it) != imu_data.end()) {
    CHECK_GT(std::next(*it)->time, start_time);
  }

  common::Time current_time = start_time;

  IntegrateImuResult<T> result = {Eigen::Matrix<T, 3, 1>::Zero(),
                                  Eigen::Matrix<T, 3, 1>::Zero(),
                                  Eigen::Quaterniond::Identity().cast<T>()};
  while (current_time < end_time) {
    common::Time next_imu_data = common::Time::max();
    if (std::next(*it) != imu_data.end()) {
      next_imu_data = std::next(*it)->time;
    }
    common::Time next_time = std::min(next_imu_data, end_time);
    const T delta_t(common::ToSeconds(next_time - current_time));

    const Eigen::Matrix<T, 3, 1> delta_angle =
        (angular_velocity_calibration *
         (*it)->angular_velocity.template cast<T>()) *
        delta_t;
    result.delta_rotation *=
        transform::AngleAxisVectorToRotationQuaternion(delta_angle);
    result.delta_velocity += result.delta_rotation *
                             ((linear_acceleration_calibration *
                               (*it)->linear_acceleration.template cast<T>()) *
                              delta_t);
    result.delta_translation += delta_t * result.delta_velocity;
    current_time = next_time;
    if (current_time == next_imu_data) {
      ++(*it);
    }
  }
  return result;
}

#endif