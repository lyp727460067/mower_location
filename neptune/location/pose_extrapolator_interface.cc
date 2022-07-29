
#include "pose_extrapolator_interface.h"
#include "location/pose_extrapolator.h"

namespace neptune {
namespace location {
std::unique_ptr<PoseExtrapolatorInterface>
PoseExtrapolatorInterface::CreateWithImuData(
    const std::vector<sensor::ImuData>& imu_data) {
  return PoseExtrapolator::InitializeWithImu(common::FromSeconds(0.001), 10,
                                             imu_data.back());
}
}}