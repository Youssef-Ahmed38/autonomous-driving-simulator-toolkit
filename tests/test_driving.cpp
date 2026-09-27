#include "autonomous_driving/driving.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace autonomous_driving;
using research::Vec2;

int failures = 0;

void require(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    ++failures;
  }
}

bool near(const double a, const double b, const double tolerance) { return std::abs(a - b) <= tolerance; }

void test_math() {
  require(near(research::wrap_angle(3.0 * std::numbers::pi), std::numbers::pi, 1e-12), "wrap 3pi -> pi");
  require(near(research::wrap_angle(-3.0 * std::numbers::pi / 2.0), std::numbers::pi / 2.0, 1e-12), "wrap -3pi/2 -> pi/2");
  require(near(research::cross({1, 0}, {0, 1}), 1.0, 1e-12), "cross sign");
  Matrix<2, 2> m;
  m(0, 0) = 4; m(0, 1) = 7; m(1, 0) = 2; m(1, 1) = 6;
  const auto product = m * inverse(m);
  require(near(product(0, 0), 1, 1e-12) && near(product(0, 1), 0, 1e-12) && near(product(1, 1), 1, 1e-12), "2x2 inverse");
}

void test_vehicle_model() {
  const KinematicBicycleModel model;
  VehicleState state;
  state.velocity = {10.0, 0.0};
  for (int i = 0; i < 100; ++i) state = model.step(state, {}, 0.01);
  require(near(state.position.x, 10.0, 1e-9) && near(state.position.y, 0.0, 1e-9), "straight driving covers v*t");

  // Constant steering traces a circle of radius L / tan(delta).
  const double steering = 0.2;
  const double radius = model.limits().wheelbase / std::tan(steering);
  state = {};
  state.velocity = {5.0, 0.0};
  double max_error = 0.0;
  for (int i = 0; i < 2000; ++i) {
    state = model.step(state, {steering, 0.0}, 0.01);
    max_error = std::max(max_error, std::abs(research::norm(state.position - Vec2{0.0, radius}) - radius));
  }
  require(max_error < 1e-3, "bicycle model follows the turning circle");

  state = {};
  state.velocity = {1.0, 0.0};
  state = model.step(state, {0.0, -9.0}, 1.0);
  require(state.velocity.x >= 0.0 && near(research::norm(state.velocity), 0.0, 1e-12), "braking never reverses");
}

void test_alpha_beta() {
  AlphaBetaSensorFusion fusion;
  const auto first = fusion.update({3.0, 4.0}, 0.5, 0.1);
  require(near(first.position.x, 3.0, 1e-12) && near(first.uncertainty, 0.5, 1e-12), "first measurement initializes");
  std::mt19937 rng(1);
  // With the default (aggressive) gains the velocity estimate is noisy, so judge its average.
  std::normal_distribution<double> noise(0.0, 0.1);
  VehicleState estimate;
  Vec2 velocity_sum;
  for (int i = 1; i <= 400; ++i) {
    estimate = fusion.update({3.0 + 2.0 * i * 0.1 + noise(rng), 4.0 + noise(rng)}, 0.1, 0.1);
    if (i > 100) velocity_sum += estimate.velocity;
  }
  const auto mean_velocity = velocity_sum * (1.0 / 300.0);
  require(estimate.uncertainty < 0.1, "propagated uncertainty drops below measurement noise");
  require(near(mean_velocity.x, 2.0, 0.1) && near(mean_velocity.y, 0.0, 0.1), "alpha-beta recovers velocity");
  require(near(estimate.heading_radians, 0.0, 0.4), "heading follows velocity");
}

void test_kalman_filter() {
  ConstantVelocityKalmanFilter filter;
  filter.initialize({0.0, 0.0}, 2.0);
  std::mt19937 rng(2);
  std::normal_distribution<double> noise(0.0, 1.0);
  const Vec2 velocity{3.0, -1.0};
  const double initial_sigma = filter.position_sigma();
  for (int i = 1; i <= 200; ++i) {
    filter.predict(0.1);
    const auto truth = velocity * (i * 0.1);
    require(filter.update_position(truth + Vec2{noise(rng), noise(rng)}, 1.0), "inlier accepted");
  }
  const auto truth = velocity * 20.0;
  require(research::norm(filter.position() - truth) < 1.0, "KF tracks position");
  require(research::norm(filter.velocity() - velocity) < 0.5, "KF estimates velocity from positions only");
  require(filter.position_sigma() < 1.0 && filter.position_sigma() < initial_sigma, "covariance shrinks");
  const auto& p = filter.covariance();
  require(near(p(0, 2), p(2, 0), 1e-12) && near(p(1, 3), p(3, 1), 1e-12), "covariance stays symmetric");

  const auto before = filter.position();
  require(!filter.update_position(truth + Vec2{50.0, 0.0}, 1.0), "50 m GNSS jump is gated out");
  require(research::norm(filter.position() - before) < 1e-12, "rejected measurement leaves state untouched");
  require(filter.update_velocity(velocity, 0.1), "velocity update accepted");

  // A persistent jump (e.g. a relocalization) must eventually be accepted instead of locking the filter out.
  bool recovered = false;
  for (int i = 0; i < 10 && !recovered; ++i) recovered = filter.update_position(truth + Vec2{50.0, 0.0}, 1.0);
  require(recovered && research::norm(filter.position() - (truth + Vec2{50.0, 0.0})) < 2.0, "gate lockout recovers");

  // Known acceleration enters the prediction.
  ConstantVelocityKalmanFilter braking;
  braking.initialize({0.0, 0.0}, 0.5, {10.0, 0.0});
  braking.predict(1.0, {-4.0, 0.0});
  require(near(braking.position().x, 8.0, 1e-9) && near(braking.velocity().x, 6.0, 1e-9), "control input in predict");
}

void test_collision_probability() {
  for (const double s : {0.3, 1.0, 2.5}) {
    const double expected = 1.0 - std::exp(-(2.0 * 2.0) / (2.0 * s * s));
    require(near(disc_collision_probability(0.0, s, 2.0), expected, 1e-4), "centred disc matches closed form");
  }
  require(disc_collision_probability(30.0, 0.5, 2.0) < 1e-9, "distant obstacle has no risk");
  require(disc_collision_probability(0.5, 1e-12, 2.0) == 1.0, "certain overlap");

  // Off-centre case against Monte Carlo.
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 1.2);
  int inside = 0;
  constexpr int samples = 400000;
  for (int i = 0; i < samples; ++i) {
    const Vec2 point{3.0 + noise(rng), noise(rng)};
    if (research::norm(point) < 2.5) ++inside;
  }
  const double monte_carlo = static_cast<double>(inside) / samples;
  require(near(disc_collision_probability(3.0, 1.2, 2.5), monte_carlo, 0.005), "off-centre probability matches Monte Carlo");
}

void test_time_to_contact_and_prediction() {
  require(near(time_to_contact({20.0, 0.0}, {-10.0, 0.0}, 2.0, 5.0), 1.8, 1e-12), "head-on TTC");
  require(std::isinf(time_to_contact({20.0, 0.0}, {10.0, 0.0}, 2.0, 5.0)), "separating objects never touch");
  require(std::isinf(time_to_contact({20.0, 5.0}, {-10.0, 0.0}, 2.0, 5.0)), "passing at 5 m misses a 2 m radius");
  require(time_to_contact({1.0, 0.0}, {}, 2.0, 5.0) == 0.0, "already overlapping");

  const ConstantVelocityPredictor predictor(1.0);
  const auto states = predictor.trajectory({{0.0, 0.0}, {2.0, 0.0}, 1.0, 0.3}, 2.0, 0.5);
  require(states.size() == 5, "trajectory sample count");
  require(near(states.back().position.x, 4.0, 1e-12), "constant velocity extrapolation");
  bool growing = true;
  for (std::size_t i = 1; i < states.size(); ++i) growing = growing && states[i].sigma > states[i - 1].sigma;
  require(growing && near(states.front().sigma, 0.3, 1e-12), "prediction uncertainty grows from the initial sigma");
}

void test_risk_assessor() {
  const RiskAssessor assessor;
  VehicleState ego;
  ego.velocity = {10.0, 0.0};
  ego.uncertainty = 0.3;
  const std::vector<Obstacle> ahead{{{30.0, 0.0}, {0.0, 0.0}, 1.0, 0.3}};
  const auto blocked = assessor.assess(ego, ahead, 4.0);
  require(blocked.risk > 0.8, "stationary car in lane is high risk");
  require(near(blocked.time_to_collision, (30.0 - 2.4) / 10.0, 1e-9), "risk TTC uses body radii");

  const std::vector<Obstacle> far_lane{{{30.0, 12.0}, {10.0, 0.0}, 1.0, 0.3}};
  require(assessor.assess(ego, far_lane, 4.0).risk < 0.01, "parallel traffic far away is low risk");

  auto uncertain = ahead;
  uncertain[0].position = {30.0, 4.5};
  auto certain_ego = ego;
  certain_ego.uncertainty = 0.05;
  uncertain[0].uncertainty = 0.05;
  const double low = assessor.assess(certain_ego, uncertain, 4.0).risk;
  uncertain[0].uncertainty = 2.0;
  const double high = assessor.assess(certain_ego, uncertain, 4.0).risk;
  require(high > low, "more perception uncertainty means more risk for a near miss");
}

void test_planner() {
  const RiskAwarePlanner planner;
  VehicleState ego;
  ego.velocity = {5.0, 0.0};

  const auto no_route = planner.plan(ego, {}, {});
  require(no_route.acceleration < 0.0 && no_route.steering == 0.0, "no route: brake and hold the wheel");

  const std::vector<Waypoint> left_turn{{{0, 0}, 10}, {{10, 0}, 10}, {{10, 30}, 10}};
  ego.position = {9.0, 0.0};
  require(planner.plan(ego, {}, left_turn).steering > 0.1, "steers left into a left turn");

  const std::vector<Waypoint> straight{{{0, 0}, 10}, {{50, 0}, 10}, {{100, 0}, 0}};
  ego.position = {0.0, 2.0};
  require(planner.plan(ego, {}, straight).steering < 0.0, "corrects back toward the route from the left");
  require(planner.progress(ego, straight).cross_track_error > 1.9, "cross-track error sign: left positive");

  ego.position = {60.0, 0.0};
  const auto progress = planner.progress(ego, straight);
  require(progress.segment == 1 && progress.lookahead_point.x > 60.0, "progress advances past earlier waypoints");
  require(near(progress.distance_to_goal, 40.0, 1e-9), "distance to goal along route");

  ego.position = {0.0, 0.0};
  ego.velocity = {0.0, 0.0};
  require(planner.plan(ego, {}, straight).acceleration > 0.0, "accelerates toward the reference speed");

  ego.position = {99.5, 0.0};
  ego.velocity = {3.0, 0.0};
  require(planner.plan(ego, {}, straight).acceleration < 0.0, "brakes at the goal");

  ego.position = {20.0, 0.0};
  ego.velocity = {10.0, 0.0};
  const std::vector<Obstacle> close{{{32.0, 0.0}, {0.0, 0.0}, 1.0, 0.2}};
  const auto emergency = planner.plan(ego, close, straight);
  require(emergency.emergency_brake && emergency.acceleration <= -planner.config().max_deceleration + 1e-9,
          "emergency brake for an imminent collision");
  require(emergency.risk > 0.8, "risk reported with the control");

  // Following a slower lead in the lane: no collision course yet, but IDM must slow the ego.
  ego.position = {20.0, 0.0};
  ego.velocity = {10.0, 0.0};
  const std::vector<Obstacle> lead{{{35.0, 0.3}, {6.0, 0.0}, 1.4, 0.2}};
  require(planner.plan(ego, lead, straight).acceleration < -1.0, "IDM brakes behind a slower, close lead");
  const std::vector<Obstacle> next_lane{{{35.0, 3.7}, {6.0, 0.0}, 1.0, 0.2}};
  const auto passing = planner.plan(ego, next_lane, straight);
  require(passing.acceleration > -1.0, "adjacent-lane car is not treated as a lead (a=" + std::to_string(passing.acceleration) +
          ", risk=" + std::to_string(passing.risk) + ")");
}

void test_closed_loop() {
  for (const auto& scenario : scenarios::all()) {
    KinematicSimulator simulator(scenario);
    DrivingStack stack;
    const auto result = run_episode(simulator, stack, 0.05, scenario.time_limit_seconds);
    require(!result.collided, scenario.name + ": no collision");
    require(result.mean_position_error < 0.5, scenario.name + ": fused position error below GNSS noise");
    if (scenario.name == "lead_vehicle_braking") {
      require(simulator.min_clearance() > 0.5, scenario.name + ": keeps a gap to the stopped car");
      require(research::norm(simulator.ground_truth()->velocity) < 0.5, scenario.name + ": ends stopped");
    } else {
      require(result.completed, scenario.name + ": reaches the goal");
    }
  }

  // The pedestrian must actually force a slowdown, otherwise the scenario tests nothing.
  KinematicSimulator crossing(scenarios::pedestrian_crossing());
  DrivingStack stack;
  double slowest_while_crossing = 1.0e9;
  double fastest_after_crossing = 0.0;
  const auto result = run_episode(crossing, stack, 0.05, 45.0, [&](const StepRecord& step) {
    const double speed = research::norm(step.truth->velocity);
    if (step.time >= 4.0 && step.time <= 9.0) slowest_while_crossing = std::min(slowest_while_crossing, speed);
    if (step.time > 9.0) fastest_after_crossing = std::max(fastest_after_crossing, speed);
  });
  require(result.max_risk > 0.3, "pedestrian scenario produces meaningful risk");
  require(slowest_while_crossing < 6.0, "ego slows well below cruise speed while the pedestrian crosses");
  require(fastest_after_crossing > 10.0, "ego resumes cruise speed once the lane is clear");
}

void test_bridge() {
  BridgeSession session;
  require(session.handle_line("frame 0 0 0 0.5 0 0.1 0 0").starts_with("error"), "frame before route is rejected");
  require(session.handle_line("route 3 0 0 10 50 0 10 100 0 0") == "ok route 3", "route accepted");
  require(session.handle_line("route 2 0 0").starts_with("error"), "truncated route rejected");
  require(session.handle_line("bogus").starts_with("error unknown command"), "unknown command");
  require(session.handle_line("").starts_with("error"), "empty line");

  const auto response = session.handle_line("frame 0 0 0 0.5 5 0.1 0 1 12 0 0 0 1.0 0.2");
  std::istringstream in(response);
  std::string word;
  double steering = 0, acceleration = 0, risk = 0, ttc = 0, x = 0, y = 0, sigma = 0;
  int emergency = 0;
  in >> word >> steering >> acceleration >> risk >> ttc >> emergency >> x >> y >> sigma;
  require(word == "control" && !in.fail(), "control response parses: " + response);
  require(risk > 0.5 && ttc > 0.0 && acceleration < 0.0, "bridge reports risk and brakes: " + response);
  require(session.handle_line("frame 0.05 0 0 0 5 0.1 0 0").starts_with("error"), "non-positive sigma rejected");
  require(session.handle_line("reset") == "ok reset", "reset");
  require(session.handle_line("quit") == "bye" && session.finished(), "quit");
}

}  // namespace

int main() {
  test_math();
  test_vehicle_model();
  test_alpha_beta();
  test_kalman_filter();
  test_collision_probability();
  test_time_to_contact_and_prediction();
  test_risk_assessor();
  test_planner();
  test_closed_loop();
  test_bridge();
  if (failures > 0) {
    std::cerr << failures << " check(s) failed.\n";
    return 1;
  }
  std::cout << "All driving toolkit tests passed.\n";
  return 0;
}
