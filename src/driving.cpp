#include "autonomous_driving/driving.hpp"

#include <algorithm>
#include <cmath>

namespace autonomous_driving {
VehicleState AlphaBetaSensorFusion::update(const research::Vec2 measured, const double uncertainty, const double dt) {
  if (!initialized_) { estimate_.position = measured; estimate_.uncertainty = uncertainty; initialized_ = true; return estimate_; }
  const auto predicted = estimate_.position + estimate_.velocity * dt; const auto residual = measured - predicted; const double trust = std::clamp(1.0 - uncertainty, 0.05, 1.0); estimate_.position = predicted + residual * (alpha_ * trust); if (dt > 1.0e-6) estimate_.velocity = estimate_.velocity + residual * (beta_ * trust / dt); estimate_.uncertainty = uncertainty; return estimate_;
}

Control RiskAwarePlanner::plan(const VehicleState& ego, const std::span<const Obstacle> obstacles, const std::span<const Waypoint> route, const double horizon) const {
  Control control; if (route.empty()) return control; const auto target = route.front(); const auto delta = target.position - ego.position; const double desired_heading = std::atan2(delta.y, delta.x); double heading_error = desired_heading - ego.heading_radians; while (heading_error > 3.141592653589793) heading_error -= 6.283185307179586; while (heading_error < -3.141592653589793) heading_error += 6.283185307179586; control.steering = std::clamp(heading_error, -0.6, 0.6);
  double minimum_margin = 1.0; for (const auto& obstacle : obstacles) { const auto relative_position = obstacle.position - ego.position; const auto relative_velocity = obstacle.velocity - ego.velocity; const double speed_squared = research::dot(relative_velocity, relative_velocity); const double closest_time = speed_squared > 1.0e-8 ? std::clamp(-research::dot(relative_position, relative_velocity) / speed_squared, 0.0, horizon) : 0.0; const double distance = research::norm(relative_position + relative_velocity * closest_time) - obstacle.radius; const double safety_radius = 2.0 + 3.0 * (ego.uncertainty + obstacle.uncertainty); minimum_margin = std::min(minimum_margin, std::clamp(distance / safety_radius, 0.0, 1.0)); }
  control.risk = 1.0 - minimum_margin; const double speed = research::norm(ego.velocity); const double safe_target_speed = target.target_speed * minimum_margin; control.acceleration = std::clamp((safe_target_speed - speed) * 0.8, -6.0, 2.5); return control;
}
}  // namespace autonomous_driving
