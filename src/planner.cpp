#include "autonomous_driving/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomous_driving {

RouteProgress RiskAwarePlanner::progress(const VehicleState& ego, const std::span<const Waypoint> route) const {
  RouteProgress result;
  if (route.empty()) {
    result.finished = true;
    return result;
  }
  if (route.size() == 1) {
    result.closest_point = result.lookahead_point = route.front().position;
    result.distance_to_goal = research::norm(route.front().position - ego.position);
    result.reference_speed = route.front().target_speed;
    result.finished = result.distance_to_goal < config_.goal_tolerance;
    return result;
  }

  // Closest point over all segments (stateless, so it tolerates frame drops and resets).
  double best_distance = std::numeric_limits<double>::infinity();
  double best_fraction = 0.0;
  for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    const auto start = route[i].position;
    const auto segment = route[i + 1].position - start;
    const double length_squared = research::dot(segment, segment);
    const double fraction = length_squared > 1.0e-12
        ? std::clamp(research::dot(ego.position - start, segment) / length_squared, 0.0, 1.0) : 0.0;
    const auto candidate = start + segment * fraction;
    const double distance = research::norm(ego.position - candidate);
    if (distance < best_distance) {
      best_distance = distance;
      best_fraction = fraction;
      result.segment = i;
      result.closest_point = candidate;
    }
  }

  const auto& from = route[result.segment];
  const auto& to = route[result.segment + 1];
  const auto direction = research::normalized(to.position - from.position);
  result.cross_track_error = research::cross(direction, ego.position - result.closest_point);
  result.reference_speed = from.target_speed + (to.target_speed - from.target_speed) * best_fraction;

  result.distance_to_goal = research::norm(to.position - result.closest_point);
  for (std::size_t i = result.segment + 1; i + 1 < route.size(); ++i)
    result.distance_to_goal += research::norm(route[i + 1].position - route[i].position);
  result.finished = result.distance_to_goal < config_.goal_tolerance;

  // Walk the lookahead distance forward along the polyline.
  const double speed = research::norm(ego.velocity);
  double remaining = config_.lookahead_base + config_.lookahead_gain * speed;
  auto cursor = result.closest_point;
  result.lookahead_point = route.back().position;
  for (std::size_t i = result.segment + 1; i < route.size(); ++i) {
    const auto next = route[i].position;
    const double length = research::norm(next - cursor);
    if (length >= remaining) {
      result.lookahead_point = cursor + research::normalized(next - cursor) * remaining;
      break;
    }
    remaining -= length;
    cursor = next;
  }
  return result;
}

Control RiskAwarePlanner::plan(const VehicleState& ego, const std::span<const Obstacle> obstacles,
                               const std::span<const Waypoint> route, const double horizon) const {
  Control control;
  const double speed = research::norm(ego.velocity);
  const auto risk = assessor_.assess(ego, obstacles, horizon);
  control.risk = risk.risk;
  control.time_to_collision = risk.time_to_collision;

  double reference_speed = 0.0;
  double goal_acceleration = 0.0;
  if (!route.empty()) {
    const auto route_progress = progress(ego, route);
    // Pure pursuit toward the lookahead point.
    const auto to_target = route_progress.lookahead_point - ego.position;
    const double distance = research::norm(to_target);
    double curvature = 0.0;
    if (distance > 1.0e-3 && !route_progress.finished) {
      const double alpha = research::wrap_angle(std::atan2(to_target.y, to_target.x) - ego.heading_radians);
      curvature = 2.0 * std::sin(alpha) / distance;
      control.steering = std::clamp(std::atan(config_.wheelbase * curvature), -config_.max_steering, config_.max_steering);
    }

    reference_speed = route_progress.finished ? 0.0 : route_progress.reference_speed;
    if (std::abs(curvature) > 1.0e-6)
      reference_speed = std::min(reference_speed, std::sqrt(config_.max_lateral_acceleration / std::abs(curvature)));
    // Arrive at the goal able to stop within the comfort deceleration.
    const double stopping_room = std::max(0.0, route_progress.distance_to_goal - 0.5 * config_.goal_tolerance);
    reference_speed = std::min(reference_speed, std::sqrt(2.0 * config_.comfort_deceleration * stopping_room));
    // Feed-forward: once stopping needs half the comfort deceleration, brake at exactly the required rate.
    const double required = speed * speed / (2.0 * std::max(stopping_room, 0.05));
    if (route_progress.finished) goal_acceleration = -config_.comfort_deceleration;
    else if (required >= 0.5 * config_.comfort_deceleration) goal_acceleration = -required;
  }

  // Uncertainty-aware speed: above the tolerance, scale the target down linearly to a stop at risk 1.
  const double excess_risk = std::clamp((risk.risk - config_.risk_tolerance) / std::max(1.0 - config_.risk_tolerance, 1.0e-6), 0.0, 1.0);
  const double safe_speed = reference_speed * (1.0 - excess_risk);
  double acceleration = std::clamp(config_.speed_gain * (safe_speed - speed), -config_.comfort_deceleration,
                                   config_.max_acceleration);
  if (goal_acceleration < 0.0 && speed > 0.05) acceleration = std::min(acceleration, goal_acceleration);

  // Car following (IDM interaction term) against the nearest obstacle inside the lane corridor ahead.
  const auto forward = research::from_heading(ego.heading_radians);
  double lead_gap = kNoCollision;
  double lead_speed = 0.0;
  for (const auto& obstacle : obstacles) {
    const auto relative = obstacle.position - ego.position;
    const double longitudinal = research::dot(relative, forward);
    const double lateral = std::abs(research::cross(forward, relative));
    const double gap = longitudinal - config_.risk.ego_radius - obstacle.radius;
    if (longitudinal > 0.0 && lateral < config_.lane_half_width + obstacle.radius && gap < lead_gap) {
      lead_gap = gap;
      lead_speed = research::dot(obstacle.velocity, forward);
    }
  }
  if (std::isfinite(lead_gap)) {
    const double closing = speed - lead_speed;
    const double desired_gap = config_.standstill_gap + speed * config_.following_time_gap +
        speed * closing / (2.0 * std::sqrt(config_.max_acceleration * config_.comfort_deceleration));
    const double ratio = std::max(desired_gap, 0.0) / std::max(lead_gap, 0.1);
    acceleration = std::min(acceleration, config_.max_acceleration * (1.0 - ratio * ratio));
  }
  if (std::isfinite(risk.time_to_collision) && speed > 0.1) {
    // Decelerate enough to stop within the distance the ego would cover before contact.
    const double required = speed / (2.0 * std::max(risk.time_to_collision, 1.0e-3));
    acceleration = std::min(acceleration, -std::min(required, config_.max_deceleration));
    if (risk.time_to_collision < config_.emergency_ttc) {
      acceleration = -config_.max_deceleration;
      control.emergency_brake = true;
    }
  }
  control.acceleration = std::clamp(acceleration, -config_.max_deceleration, config_.max_acceleration);
  return control;
}

}  // namespace autonomous_driving
