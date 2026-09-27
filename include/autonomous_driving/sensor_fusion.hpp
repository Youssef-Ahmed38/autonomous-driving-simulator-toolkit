#pragma once

#include "autonomous_driving/matrix.hpp"
#include "autonomous_driving/types.hpp"

namespace autonomous_driving {

/// Lightweight alpha-beta tracker. The gains are scaled down for noisy measurements and the
/// estimate's 1-sigma position error is propagated rather than copied from the measurement.
class AlphaBetaSensorFusion {
 public:
  AlphaBetaSensorFusion(double alpha = 0.75, double beta = 0.20, double process_sigma = 1.0)
      : alpha_(alpha), beta_(beta), process_sigma_(process_sigma) {}

  /// `measurement_uncertainty` is the 1-sigma position error of the measurement in meters.
  [[nodiscard]] VehicleState update(research::Vec2 measured_position, double measurement_uncertainty,
                                    double delta_seconds);
  void reset() { initialized_ = false; estimate_ = {}; }

 private:
  double alpha_;
  double beta_;
  double process_sigma_;
  bool initialized_ = false;
  VehicleState estimate_{};
};

struct KalmanConfig {
  double acceleration_noise = 1.5;    ///< White-acceleration spectral density sigma, m/s^2.
  double initial_velocity_sigma = 5.0;
  double gate_threshold = 13.82;      ///< Chi-square (2 dof) 99.9 % gate on normalized innovation squared.
  int max_consecutive_rejections = 5; ///< After this many gated updates in a row the filter re-opens its covariance.
};

/// Constant-velocity Kalman filter over [px, py, vx, vy] with position (GNSS) and velocity
/// (wheel odometry + heading) updates and Mahalanobis outlier gating.
class ConstantVelocityKalmanFilter {
 public:
  using StateVector = Matrix<4, 1>;
  using Covariance = Matrix<4, 4>;

  explicit ConstantVelocityKalmanFilter(KalmanConfig config = {}) : config_(config) {}

  void initialize(research::Vec2 position, double position_sigma, research::Vec2 velocity = {});
  /// `acceleration` is the known (commanded) acceleration in the world frame, if any.
  void predict(double delta_seconds, research::Vec2 acceleration = {});
  /// Returns false (and leaves the state untouched) when the measurement fails the gate. Repeated
  /// rejections mean the filter, not the sensor, is wrong: the covariance is inflated and the
  /// measurement is accepted so the filter cannot lock itself out.
  bool update_position(research::Vec2 measured, double sigma);
  bool update_velocity(research::Vec2 measured, double sigma);

  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] research::Vec2 position() const { return {x_(0, 0), x_(1, 0)}; }
  [[nodiscard]] research::Vec2 velocity() const { return {x_(2, 0), x_(3, 0)}; }
  [[nodiscard]] const Covariance& covariance() const { return p_; }
  /// 1-sigma position error along the worst axis.
  [[nodiscard]] double position_sigma() const;
  [[nodiscard]] double last_innovation_score() const { return last_nis_; }
  void reset() { initialized_ = false; }

 private:
  bool update(const Matrix<2, 4>& h, research::Vec2 measured, double sigma, int& consecutive_rejections);

  KalmanConfig config_;
  bool initialized_ = false;
  StateVector x_{};
  Covariance p_{};
  double last_nis_ = 0.0;
  int position_rejections_ = 0;
  int velocity_rejections_ = 0;
};

}  // namespace autonomous_driving
