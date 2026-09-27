#include "autonomous_driving/driving_stack.hpp"

#include <algorithm>
#include <cmath>

namespace autonomous_driving {

Control DrivingStack::on_frame(const SensorFrame& frame, const std::span<const Waypoint> route) {
  const auto measured_velocity = research::from_heading(frame.heading_radians, frame.speed);
  // Heading noise turns into lateral velocity error proportional to speed.
  const double velocity_sigma = std::hypot(frame.speed_sigma, 0.02 * frame.speed);
  if (!filter_.initialized()) {
    filter_.initialize(frame.gnss_position, frame.gnss_sigma, measured_velocity);
  } else {
    filter_.predict(frame.timestamp - last_timestamp_, research::from_heading(frame.heading_radians, last_acceleration_));
    if (!filter_.update_position(frame.gnss_position, frame.gnss_sigma)) ++rejected_;
    if (!filter_.update_velocity(measured_velocity, velocity_sigma)) ++rejected_;
  }
  last_timestamp_ = frame.timestamp;

  estimate_.position = filter_.position();
  estimate_.velocity = filter_.velocity();
  estimate_.heading_radians = frame.heading_radians;
  estimate_.uncertainty = filter_.position_sigma();
  const auto control = planner_.plan(estimate_, frame.obstacles, route, config_.horizon_seconds);
  last_acceleration_ = control.acceleration;
  return control;
}

void DrivingStack::reset() {
  filter_.reset();
  estimate_ = {};
  last_timestamp_ = 0.0;
  last_acceleration_ = 0.0;
  rejected_ = 0;
}

EpisodeResult run_episode(SimulatorAdapter& simulator, DrivingStack& stack, const double dt,
                          const double time_limit, const std::function<void(const StepRecord&)>& on_step) {
  EpisodeResult result;
  stack.reset();
  auto frame = simulator.reset();
  const auto route = simulator.route();
  double error_sum = 0.0;
  std::size_t error_samples = 0;
  while (!simulator.done() && result.duration_seconds < time_limit) {
    const auto control = stack.on_frame(frame, route);
    const auto truth = simulator.ground_truth();
    if (truth) {
      const double error = research::norm(stack.estimate().position - truth->position);
      error_sum += error;
      ++error_samples;
      result.max_position_error = std::max(result.max_position_error, error);
    }
    result.max_risk = std::max(result.max_risk, control.risk);
    if (control.emergency_brake) ++result.emergency_brakes;
    if (on_step) on_step({result.duration_seconds, &frame, stack.estimate(), truth, control});

    frame = simulator.step(control, dt);
    ++result.steps;
    result.duration_seconds += dt;
    if (frame.collision) result.collided = true;
  }
  result.collided = result.collided || frame.collision;
  result.completed = !result.collided && simulator.goal_reached();
  if (error_samples > 0) result.mean_position_error = error_sum / static_cast<double>(error_samples);
  return result;
}

}  // namespace autonomous_driving
