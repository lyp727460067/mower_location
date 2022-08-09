#include "gps_processor.h"
#include "glog/logging.h"
#include "utils.h"
namespace neptune {
namespace location {

GpsProcessor::GpsProcessor(const Eigen::Vector3d &I_p_Gps)
    : I_p_Gps_(I_p_Gps) {}

bool GpsProcessor::UpdateStateByGpsPosition(
    const Eigen::Vector3d &init_lla, const GpsPositionDataPtr gps_data_ptr,
    State *state) {
  if (gps_data_ptr->timestamp < state->timestamp) {
    LOG(WARNING) << "gps data timestamp jump back from " << state->timestamp
                 << " to " << gps_data_ptr->timestamp;
    return false;
  }

  // LOG(INFO) << "update with gps: " << *gps_data_ptr;
  // LOG(INFO) << "befor update: " << *state;
  Eigen::Matrix<double, 3, 18> H;
  Eigen::Vector3d residual;
  ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
  const Eigen::Matrix3d &V = gps_data_ptr->cov;

  // EKF.
  const Eigen::MatrixXd &P = state->cov;
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + V).inverse();
  const Eigen::VectorXd delta_x = K * residual;

  // LOG(INFO) << "jacob H = \n" << H;
  // LOG(INFO) << "gps noise V = \n" << V;
  // LOG(INFO) << "Kalman Gain K = \n" << K;
  // LOG(INFO) << "residual = " << residual.transpose();
  // LOG(INFO) << "delta_x = " << delta_x.transpose();

  // Add delta_x to state.
  AddDeltaToState(delta_x, state);

  // Covarance.
  const Eigen::MatrixXd I_KH =
      Eigen::Matrix<double, 18, 18>::Identity() - K * H;

  state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
  state->timestamp = gps_data_ptr->timestamp;

  // LOG(INFO) << "after update with gps";
  // LOG(INFO) << *state;

  return true;
}

void GpsProcessor::ComputeJacobianAndResidual(
    const Eigen::Vector3d &init_lla, const GpsPositionDataPtr gps_data,
    const State &state, Eigen::Matrix<double, 3, 18> *jacobian,
    Eigen::Vector3d *residual) {
  const Eigen::Vector3d &G_p_I = state.G_p_I;
  const Eigen::Matrix3d &G_R_I = state.G_R_I;

  // Convert wgs84 to ENU frame.
  Eigen::Vector3d G_p_Gps;
  ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);

  // Compute residual.
  *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

  // Compute jacobian.
  jacobian->setZero();
  jacobian->block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  jacobian->block<3, 3>(0, 6) = -GetSkewMatrix(G_R_I * I_p_Gps_);
}

} // namespace location
} // namespace neptune