#include "erl_quadrotor_mocap/filter.hpp"
#include <eigen3/Eigen/LU>  // For matrix inverse

namespace erl_quadrotor_mocap {

void Filter::initialize(const State_t &state, const ProcessCov_t &initial_cov,
                              const ProcessCov_t &process_noise, const MeasurementCov_t &meas_noise) {
  x = state;
  P = initial_cov;
  Q = process_noise;
  R = meas_noise;
}

void Filter::processUpdate(double dt) {
  ProcessCov_t A = ProcessCov_t::Identity();
  A.topRightCorner<3, 3>() = Eigen::Vector3d(dt, dt, dt).asDiagonal();

  x = A * x;
  P = A * P * A.transpose() + Q;
}

void Filter::measurementUpdate(const Measurement_t &meas, double dt) {
  Eigen::Matrix<double, n_meas, n_states> H = Eigen::Matrix<double, n_meas, n_states>::Zero();
  H(0, 0) = H(1, 1) = H(2, 2) = 1;

  const Eigen::Matrix<double, n_states, n_meas> K =
      P * H.transpose() * (H * P * H.transpose() + R).inverse();
  const Measurement_t inno = meas - H * x;
  x += K * inno;
  P = (ProcessCov_t::Identity() - K * H) * P;
}

}  // namespace erl_quadrotor_mocap
