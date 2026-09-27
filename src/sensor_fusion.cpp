#include "autonomous_driving/sensor_fusion.hpp"

#include <algorithm>
#include <cmath>

namespace autonomous_driving {

VehicleState AlphaBetaSensorFusion::update(const research::Vec2 measured, const double sigma, double dt) {
  if (!initialized_) {
    estimate_ = {};
    estimate_.position = measured;
    estimate_.uncertainty = sigma;
    initialized_ = true;
    return estimate_;
  }
  dt = std::max(dt, 0.0);
  const auto predicted = estimate_.position + estimate_.velocity * dt;
  // Unknown acceleration makes the prediction drift by roughly a * dt^2 / 2.
  const double predicted_sigma = std::hypot(estimate_.uncertainty, 0.5 * process_sigma_ * dt * dt);
  const double trust = std::clamp(1.0 / (1.0 + sigma * sigma), 0.05, 1.0);
  const double gain = alpha_ * trust;
  const auto residual = measured - predicted;
  estimate_.position = predicted + residual * gain;
  if (dt > 1.0e-6) estimate_.velocity = estimate_.velocity + residual * (beta_ * trust / dt);
  // Convex blend of two independent estimates.
  estimate_.uncertainty = std::hypot((1.0 - gain) * predicted_sigma, gain * sigma);
  if (research::norm(estimate_.velocity) > 0.5) estimate_.heading_radians = std::atan2(estimate_.velocity.y, estimate_.velocity.x);
  return estimate_;
}

void ConstantVelocityKalmanFilter::initialize(const research::Vec2 position, const double position_sigma,
                                              const research::Vec2 velocity) {
  x_ = {};
  x_(0, 0) = position.x;
  x_(1, 0) = position.y;
  x_(2, 0) = velocity.x;
  x_(3, 0) = velocity.y;
  p_ = {};
  const double position_variance = position_sigma * position_sigma;
  const double velocity_variance = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
  p_(0, 0) = p_(1, 1) = position_variance;
  p_(2, 2) = p_(3, 3) = velocity_variance;
  last_nis_ = 0.0;
  position_rejections_ = velocity_rejections_ = 0;
  initialized_ = true;
}

void ConstantVelocityKalmanFilter::predict(const double dt, const research::Vec2 acceleration) {
  if (!initialized_ || dt <= 0.0) return;
  auto f = Covariance::identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  // Discrete white-noise acceleration model, independent per axis.
  const double q = config_.acceleration_noise * config_.acceleration_noise;
  const double dt2 = dt * dt;
  Covariance noise;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    noise(axis, axis) = 0.25 * dt2 * dt2 * q;
    noise(axis, axis + 2) = noise(axis + 2, axis) = 0.5 * dt2 * dt * q;
    noise(axis + 2, axis + 2) = dt2 * q;
  }
  x_ = f * x_;
  x_(0, 0) += 0.5 * acceleration.x * dt2;
  x_(1, 0) += 0.5 * acceleration.y * dt2;
  x_(2, 0) += acceleration.x * dt;
  x_(3, 0) += acceleration.y * dt;
  p_ = f * p_ * transpose(f) + noise;
}

bool ConstantVelocityKalmanFilter::update_position(const research::Vec2 measured, const double sigma) {
  Matrix<2, 4> h;
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  return update(h, measured, sigma, position_rejections_);
}

bool ConstantVelocityKalmanFilter::update_velocity(const research::Vec2 measured, const double sigma) {
  Matrix<2, 4> h;
  h(0, 2) = 1.0;
  h(1, 3) = 1.0;
  return update(h, measured, sigma, velocity_rejections_);
}

bool ConstantVelocityKalmanFilter::update(const Matrix<2, 4>& h, const research::Vec2 measured, double sigma,
                                          int& consecutive_rejections) {
  if (!initialized_) return false;
  sigma = std::max(sigma, 1.0e-6);
  Matrix<2, 1> z;
  z(0, 0) = measured.x;
  z(1, 0) = measured.y;
  const auto innovation = z - h * x_;
  auto r = Matrix<2, 2>::identity();
  r(0, 0) = r(1, 1) = sigma * sigma;
  const auto h_t = transpose(h);
  auto s_inv = inverse(h * p_ * h_t + r);
  last_nis_ = (transpose(innovation) * s_inv * innovation)(0, 0);
  if (last_nis_ > config_.gate_threshold) {
    if (++consecutive_rejections < config_.max_consecutive_rejections) return false;
    // Persistent disagreement: drop correlations and make the measured states nearly unknown,
    // so the measurement is adopted without dragging the other states along.
    Covariance reopened;
    for (std::size_t i = 0; i < 4; ++i) reopened(i, i) = p_(i, i);
    for (std::size_t i = 0; i < 4; ++i)
      if (h(0, i) != 0.0 || h(1, i) != 0.0) reopened(i, i) = std::max(100.0 * p_(i, i), 1.0e4 * sigma * sigma);
    p_ = reopened;
    s_inv = inverse(h * p_ * h_t + r);
  }
  consecutive_rejections = 0;
  const auto gain = p_ * h_t * s_inv;
  x_ = x_ + gain * innovation;
  // Joseph form keeps the covariance symmetric positive semi-definite.
  const auto i_kh = Covariance::identity() - gain * h;
  p_ = i_kh * p_ * transpose(i_kh) + gain * r * transpose(gain);
  return true;
}

double ConstantVelocityKalmanFilter::position_sigma() const {
  return std::sqrt(std::max({p_(0, 0), p_(1, 1), 0.0}));
}

}  // namespace autonomous_driving
