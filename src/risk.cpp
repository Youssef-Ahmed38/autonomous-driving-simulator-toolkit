#include "autonomous_driving/risk.hpp"

#include <algorithm>
#include <cmath>

namespace autonomous_driving {
namespace {

/// Exponentially scaled modified Bessel function exp(-x) * I0(x) for x >= 0
/// (Abramowitz & Stegun 9.8.1 / 9.8.2 polynomial approximations).
double bessel_i0_scaled(const double x) {
  if (x < 3.75) {
    const double t = (x / 3.75) * (x / 3.75);
    const double i0 = 1.0 + t * (3.5156229 + t * (3.0899424 + t * (1.2067492 + t * (0.2659732 + t * (0.0360768 + t * 0.0045813)))));
    return i0 * std::exp(-x);
  }
  const double t = 3.75 / x;
  const double poly = 0.39894228 + t * (0.01328592 + t * (0.00225319 + t * (-0.00157565 + t * (0.00916281 +
                      t * (-0.02057706 + t * (0.02635537 + t * (-0.01647633 + t * 0.00392377)))))));
  return poly / std::sqrt(x);
}

double predicted_sigma(const double initial_sigma, const double acceleration_sigma, const double time) {
  return std::hypot(initial_sigma, 0.5 * acceleration_sigma * time * time);
}

}  // namespace

PredictedState ConstantVelocityPredictor::at(const research::Vec2 position, const research::Vec2 velocity,
                                             const double initial_sigma, const double time) const {
  return {time, position + velocity * time, predicted_sigma(initial_sigma, acceleration_sigma_, time)};
}

std::vector<PredictedState> ConstantVelocityPredictor::trajectory(const Obstacle& obstacle, const double horizon,
                                                                  const double step) const {
  std::vector<PredictedState> states;
  if (step <= 0.0 || horizon < 0.0) return states;
  const auto count = static_cast<std::size_t>(std::floor(horizon / step + 1.0e-9)) + 1;
  states.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
    states.push_back(at(obstacle.position, obstacle.velocity, obstacle.uncertainty, static_cast<double>(i) * step));
  return states;
}

double disc_collision_probability(const double d, const double s, const double radius) {
  if (radius <= 0.0) return 0.0;
  if (s < 1.0e-9) return d < radius ? 1.0 : 0.0;
  // P(|X| < R) = integral_0^R Rice(r; d, s) dr. The integrand is negligible more than
  // 8 sigma from d, so integrate only that window for accuracy with narrow distributions.
  const double lower = std::max(0.0, d - 8.0 * s);
  const double upper = std::min(radius, d + 8.0 * s);
  if (upper <= lower) return d < radius ? 1.0 : 0.0;
  const double inv_var = 1.0 / (s * s);
  const auto density = [&](const double r) {
    const double gap = r - d;
    return r * inv_var * std::exp(-0.5 * gap * gap * inv_var) * bessel_i0_scaled(r * d * inv_var);
  };
  constexpr int intervals = 128;  // Simpson's rule, even count.
  const double h = (upper - lower) / intervals;
  double sum = density(lower) + density(upper);
  for (int i = 1; i < intervals; ++i) sum += density(lower + i * h) * (i % 2 == 1 ? 4.0 : 2.0);
  return std::clamp(sum * h / 3.0, 0.0, 1.0);
}

double time_to_contact(const research::Vec2 p, const research::Vec2 v, const double radius, const double horizon) {
  const double c = research::dot(p, p) - radius * radius;
  if (c <= 0.0) return 0.0;
  const double a = research::dot(v, v);
  if (a < 1.0e-12) return kNoCollision;
  const double b = 2.0 * research::dot(p, v);
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) return kNoCollision;
  const double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
  return (t >= 0.0 && t <= horizon) ? t : kNoCollision;
}

RiskAssessment RiskAssessor::assess(const VehicleState& ego, const std::span<const Obstacle> obstacles,
                                    const double horizon) const {
  RiskAssessment assessment;
  assessment.obstacles.reserve(obstacles.size());
  const double step = config_.step_seconds > 0.0 ? config_.step_seconds : 0.1;
  const auto steps = static_cast<int>(std::floor(std::max(horizon, 0.0) / step + 1.0e-9));
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const auto& obstacle = obstacles[index];
    const auto relative_position = obstacle.position - ego.position;
    const auto relative_velocity = obstacle.velocity - ego.velocity;
    const double body_radius = config_.ego_radius + obstacle.radius;
    const double contact_radius = body_radius + config_.safety_margin;

    ObstacleRisk result;
    result.index = index;
    result.min_clearance = kNoCollision;
    result.time_to_collision = time_to_contact(relative_position, relative_velocity, body_radius, horizon);
    for (int i = 0; i <= steps; ++i) {
      const double t = i * step;
      const double distance = research::norm(relative_position + relative_velocity * t);
      // The ego follows its own plan, so only its current estimate error counts; the obstacle's grows.
      const double sigma = std::hypot(ego.uncertainty, predicted_sigma(obstacle.uncertainty, config_.acceleration_sigma, t));
      const double probability = disc_collision_probability(distance, sigma, contact_radius);
      if (probability > result.collision_probability) {
        result.collision_probability = probability;
        result.time_of_peak = t;
      }
      result.min_clearance = std::min(result.min_clearance, distance - body_radius);
    }
    assessment.risk = std::max(assessment.risk, result.collision_probability);
    assessment.time_to_collision = std::min(assessment.time_to_collision, result.time_to_collision);
    assessment.min_clearance = std::min(assessment.min_clearance, result.min_clearance);
    assessment.obstacles.push_back(result);
  }
  return assessment;
}

}  // namespace autonomous_driving
