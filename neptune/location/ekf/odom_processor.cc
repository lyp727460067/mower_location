#include "odom_processor.h"
#include "glog/logging.h"
#include "utils.h"

namespace neptune {
namespace location {

OdomVelocityProcessor::OdomVelocityProcessor(const Eigen::Vector3d &I_t_odom,
                                             const Eigen::Matrix3d &I_R_odom)
    : I_t_odom_(I_t_odom), I_R_odom_(I_R_odom) {}

bool OdomVelocityProcessor::UpdateStateByOdomVelocity(
    const OdomVelocityDataPtr odom_data_ptr, State *state) {

  if (odom_data_ptr->timestamp < state->timestamp) {
    LOG(WARNING) << "odom data timestamp jump back form " << state->timestamp
                 << " to " << odom_data_ptr->timestamp;
    return false;
  }

  // LOG(INFO) << "update with odom = " << *odom_data_ptr;
  // LOG(INFO) << "before update state = " << *state;

  Eigen::Matrix<double, 6, 18> H;
  Eigen::Matrix<double, 6, 1> residual;
  ComputeJacobianAndResidual(odom_data_ptr, *state, &H, &residual);
  const Eigen::Matrix<double, 6, 6> &V = odom_data_ptr->cov;

  // EKF.
  const Eigen::Matrix<double, 18, 18> &P = state->cov;
  const Eigen::Matrix<double, 18, 6> K =
      P * H.transpose() * (H * P * H.transpose() + V).inverse();
  const Eigen::Matrix<double, 18, 1> delta_x = K * residual;

  // LOG(INFO) << "jacob H = \n" << H;
  // LOG(INFO) << "odom noise V = \n" << V;
  // LOG(INFO) << "Kalman Gain K = \n" << K;
  // LOG(INFO) << "residual = " << residual.transpose();
  // LOG(INFO) << "delta_x = " << delta_x.transpose();

  // Add delta_x to state.
  AddDeltaToState(delta_x, state);

  // Covarance.
  const Eigen::Matrix<double, 18, 18> I_KH =
      Eigen::Matrix<double, 18, 18>::Identity() - K * H;
  state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
  // state->timestamp = odom_data_ptr->timestamp;
  // LOG(INFO) << "after update with odom";
  // LOG(INFO) << *state;
  return true;
}

void OdomVelocityProcessor::ComputeJacobianAndResidual(
    const OdomVelocityDataPtr odom_data, const State &state,
    Eigen::Matrix<double, 6, 18> *jacobian,
    Eigen::Matrix<double, 6, 1> *residual) {
  jacobian->setZero();
  jacobian->block<3, 3>(0, 3) = I_R_odom_.transpose() * state.G_R_I.transpose();
  jacobian->block<3, 3>(0, 6) =
      -I_R_odom_.transpose() *
      GetSkewMatrix(state.G_R_I.transpose() * state.G_v_I);
  jacobian->block<3, 3>(0, 15) =
      -I_R_odom_.transpose() * GetSkewMatrix(I_t_odom_);
  jacobian->block<3, 3>(3, 15) = I_R_odom_.transpose();
}

} // namespace location
} // namespace neptune