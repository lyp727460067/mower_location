#ifndef ODOM_PROCESSOR_H
#define ODOM_PROCESSOR_H
#include "base_type.h"
#include <Eigen/Dense>
namespace neptune {
namespace location {

class OdomVelocityProcessor {
public:
  OdomVelocityProcessor(const Eigen::Vector3d &I_t_odom, const Eigen::Matrix3d &I_R_odom);
  bool UpdateStateByOdomVelocity(const OdomVelocityDataPtr odom_data_ptr,
                                 State *state);

private:
  void ComputeJacobianAndResidual(const OdomVelocityDataPtr odom_data,
                                  const State &state,
                                  Eigen::Matrix<double, 6, 18> *jacobian,
                                  Eigen::Matrix<double, 6, 1> *residual);

  const Eigen::Vector3d I_t_odom_;
  const Eigen::Matrix3d I_R_odom_;
};

} // namespace location
} // namespace neptune

#endif
