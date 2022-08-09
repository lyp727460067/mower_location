#ifndef GPS_PROCESSOR_H
#define GPS_PROCESSOR_H
#include <Eigen/Dense>
#include "base_type.h"
namespace neptune {
namespace location {

class GpsProcessor {
 public:
  GpsProcessor(const Eigen::Vector3d& I_p_Gps);

  bool UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla,
                                const GpsPositionDataPtr gps_data_ptr,
                                State* state);

 private:
  void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,
                                  const GpsPositionDataPtr gps_data,
                                  const State& state,
                                  Eigen::Matrix<double, 3, 18>* jacobian,
                                  Eigen::Vector3d* residual);

  const Eigen::Vector3d I_p_Gps_;
};


}  // namespace location
}  // namespace neptune

#endif