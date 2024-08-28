#ifndef ERL_QUADROTOR_MOCAP_FILTER_HPP_
#define ERL_QUADROTOR_MOCAP_FILTER_HPP_

#include <Eigen/Core>

namespace erl_quadrotor_mocap {

class Filter {
 public:
  static const int n_states = 6;
  static const int n_meas = 3;
  using State_t = Eigen::Matrix<double, n_states, 1>;
  using ProcessCov_t = Eigen::Matrix<double, n_states, n_states>;
  using Measurement_t = Eigen::Matrix<double, n_meas, 1>;
  using MeasurementCov_t = Eigen::Matrix<double, n_meas, n_meas>;

  Filter() = default;
  Filter(const State_t &state, const ProcessCov_t &initial_cov,
               const ProcessCov_t &process_noise, const MeasurementCov_t &meas_noise)
      : x(state), P(initial_cov), Q(process_noise), R(meas_noise) {}

  void initialize(const State_t &state, const ProcessCov_t &initial_cov,
                  const ProcessCov_t &process_noise, const MeasurementCov_t &meas_noise);

  void processUpdate(double dt);
  void measurementUpdate(const Measurement_t &meas, double dt);

  void setProcessNoise(const ProcessCov_t &process_noise) { Q = process_noise; }
  void setMeasurementNoise(const MeasurementCov_t &meas_noise) { R = meas_noise; }

  const State_t &getState() const { return x; }
  const ProcessCov_t &getProcessNoise() const { return P; }

 private:
  State_t x;
  ProcessCov_t P, Q;
  MeasurementCov_t R;
};

}  // namespace erl_quadrotor_mocap

#endif  // ERL_QUADROTOR_MOCAP_FILTER_HPP_
