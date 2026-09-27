#include "autonomous_driving/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace autonomous_driving {
namespace {

std::vector<Waypoint> straight_route(const double length, const double spacing, const double speed) {
  std::vector<Waypoint> route;
  for (double x = 0.0; x <= length + 1.0e-9; x += spacing) route.push_back({{x, 0.0}, speed});
  route.back().target_speed = 0.0;
  return route;
}

}  // namespace

namespace scenarios {

Scenario straight_road() {
  Scenario scenario;
  scenario.name = "straight_road";
  scenario.route = straight_route(100.0, 10.0, 12.0);
  scenario.time_limit_seconds = 40.0;
  return scenario;
}

Scenario curved_road() {
  Scenario scenario;
  scenario.name = "curved_road";
  constexpr double speed = 10.0;
  constexpr double radius = 25.0;
  for (double x = 0.0; x < 30.0; x += 5.0) scenario.route.push_back({{x, 0.0}, speed});
  // Quarter circle to the left centred on (30, radius).
  for (int i = 0; i <= 12; ++i) {
    const double angle = -std::numbers::pi / 2.0 + (std::numbers::pi / 2.0) * i / 12.0;
    scenario.route.push_back({{30.0 + radius * std::cos(angle), radius + radius * std::sin(angle)}, speed});
  }
  for (double y = radius + 5.0; y <= radius + 30.0 + 1.0e-9; y += 5.0) scenario.route.push_back({{30.0 + radius, y}, speed});
  scenario.route.back().target_speed = 0.0;
  scenario.time_limit_seconds = 40.0;
  scenario.seed = 11;
  return scenario;
}

Scenario pedestrian_crossing() {
  Scenario scenario;
  scenario.name = "pedestrian_crossing";
  scenario.route = straight_route(120.0, 10.0, 12.0);
  // Walks across the lane from the right, reaching the centreline around t = 5 s.
  scenario.actors.push_back({{{45.0, -7.0}, {0.0, 1.4}, 0.5, 0.0}});
  scenario.time_limit_seconds = 45.0;
  scenario.seed = 23;
  return scenario;
}

Scenario lead_vehicle_braking() {
  Scenario scenario;
  scenario.name = "lead_vehicle_braking";
  scenario.ego_start.velocity = {10.0, 0.0};
  scenario.route = straight_route(200.0, 10.0, 14.0);
  // A lead car 25 m ahead brakes to a standstill at t = 4 s and stays there.
  scenario.actors.push_back({{{25.0, 0.0}, {10.0, 0.0}, 1.4, 0.0}, 4.0});
  scenario.time_limit_seconds = 25.0;
  scenario.seed = 31;
  return scenario;
}

std::vector<Scenario> all() { return {straight_road(), curved_road(), pedestrian_crossing(), lead_vehicle_braking()}; }

}  // namespace scenarios

KinematicSimulator::KinematicSimulator(Scenario scenario, const VehicleLimits limits, const double ego_radius)
    : scenario_(std::move(scenario)), model_(limits), ego_radius_(ego_radius), rng_(scenario_.seed) {
  reset();
}

SensorFrame KinematicSimulator::reset() {
  ego_ = scenario_.ego_start;
  actors_.clear();
  for (const auto& actor : scenario_.actors) actors_.push_back(actor.initial);
  time_ = 0.0;
  collided_ = false;
  min_clearance_ = std::numeric_limits<double>::infinity();
  rng_.seed(scenario_.seed);
  check_collisions();
  return observe();
}

SensorFrame KinematicSimulator::step(const Control& control, const double dt) {
  if (dt <= 0.0) return observe();
  ego_ = model_.step(ego_, control, dt);
  time_ += dt;
  for (std::size_t i = 0; i < actors_.size(); ++i) {
    if (time_ >= scenario_.actors[i].stop_after_seconds) actors_[i].velocity = {};
    actors_[i].position += actors_[i].velocity * dt;
  }
  check_collisions();
  return observe();
}

bool KinematicSimulator::goal_reached() const {
  return !scenario_.route.empty() && research::norm(scenario_.route.back().position - ego_.position) < 2.0 &&
         research::norm(ego_.velocity) < 0.5;
}

bool KinematicSimulator::done() const {
  return collided_ || goal_reached() || time_ >= scenario_.time_limit_seconds;
}

std::vector<Obstacle> KinematicSimulator::actors_truth() const { return actors_; }

void KinematicSimulator::check_collisions() {
  for (const auto& actor : actors_) {
    const double clearance = research::norm(actor.position - ego_.position) - ego_radius_ - actor.radius;
    min_clearance_ = std::min(min_clearance_, clearance);
    if (clearance < 0.0) collided_ = true;
  }
}

SensorFrame KinematicSimulator::observe() {
  std::normal_distribution<double> unit(0.0, 1.0);
  SensorFrame frame;
  frame.timestamp = time_;
  frame.gnss_sigma = scenario_.gnss_sigma;
  frame.gnss_position = ego_.position + research::Vec2{unit(rng_), unit(rng_)} * scenario_.gnss_sigma;
  frame.speed_sigma = scenario_.speed_sigma;
  frame.speed = std::max(0.0, research::norm(ego_.velocity) + unit(rng_) * scenario_.speed_sigma);
  frame.heading_radians = research::wrap_angle(ego_.heading_radians + unit(rng_) * 0.01);
  frame.collision = collided_;
  frame.obstacles.reserve(actors_.size());
  for (const auto& actor : actors_) {
    Obstacle seen = actor;
    seen.position += research::Vec2{unit(rng_), unit(rng_)} * scenario_.perception_sigma;
    seen.velocity += research::Vec2{unit(rng_), unit(rng_)} * 0.2;
    seen.uncertainty = scenario_.perception_sigma;
    frame.obstacles.push_back(seen);
  }
  return frame;
}

}  // namespace autonomous_driving
