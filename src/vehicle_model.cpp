#include "autonomous_driving/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace autonomous_driving {

VehicleState KinematicBicycleModel::step(const VehicleState& state, const Control& control, const double dt) const {
  if (dt <= 0.0) return state;
  const double speed = std::max(0.0, research::dot(state.velocity, research::from_heading(state.heading_radians)));
  const double acceleration = std::clamp(control.acceleration, -limits_.max_deceleration, limits_.max_acceleration);
  const double steering = std::clamp(control.steering, -limits_.max_steering, limits_.max_steering);
  const double next_speed = std::clamp(speed + acceleration * dt, 0.0, limits_.max_speed);
  const double mean_speed = 0.5 * (speed + next_speed);
  const double yaw_rate = mean_speed / limits_.wheelbase * std::tan(steering);

  VehicleState next = state;
  // Midpoint integration of the heading keeps arcs accurate at simulation step sizes.
  next.position += research::from_heading(state.heading_radians + 0.5 * yaw_rate * dt, mean_speed * dt);
  next.heading_radians = research::wrap_angle(state.heading_radians + yaw_rate * dt);
  next.velocity = research::from_heading(next.heading_radians, next_speed);
  return next;
}

}  // namespace autonomous_driving
