#include "autonomous_driving/driving.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

template <typename Fn>
double microseconds_per_call(const int iterations, Fn&& fn) {
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) fn(i);
  const auto elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
  return elapsed / iterations;
}

}  // namespace

int main() {
  using namespace autonomous_driving;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform(-60.0, 60.0);

  std::vector<Waypoint> route;
  for (int i = 0; i < 400; ++i) route.push_back({{i * 2.5, 3.0 * std::sin(i * 0.05)}, 12.0});
  std::vector<Obstacle> obstacles;
  for (int i = 0; i < 32; ++i) obstacles.push_back({{uniform(rng) + 60.0, uniform(rng) * 0.2}, {uniform(rng) * 0.1, 0.0}, 1.0, 0.3});

  const RiskAwarePlanner planner;
  VehicleState ego;
  ego.velocity = {12.0, 0.0};
  ego.uncertainty = 0.3;
  volatile double sink = 0.0;
  const double plan_us = microseconds_per_call(2000, [&](int i) {
    ego.position.x = (i % 400) * 2.5;
    sink = sink + planner.plan(ego, obstacles, route).acceleration;
  });

  ConstantVelocityKalmanFilter filter;
  filter.initialize({0, 0}, 1.0);
  const double kf_us = microseconds_per_call(200000, [&](int i) {
    filter.predict(0.05);
    filter.update_position({i * 0.05, 0.0}, 0.5);
    filter.update_velocity({1.0, 0.0}, 0.1);
  });

  const auto episode_start = std::chrono::steady_clock::now();
  std::size_t simulated_steps = 0;
  double simulated_seconds = 0.0;
  for (const auto& scenario : scenarios::all()) {
    KinematicSimulator simulator(scenario);
    DrivingStack stack;
    const auto result = run_episode(simulator, stack, 0.05, scenario.time_limit_seconds);
    simulated_steps += result.steps;
    simulated_seconds += result.duration_seconds;
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - episode_start).count();

  std::cout << std::fixed << std::setprecision(2)
            << "planner.plan (400 waypoints, 32 obstacles, 4 s horizon): " << plan_us << " us/call\n"
            << "kalman predict + 2 updates:                            " << kf_us << " us/step\n"
            << "closed loop, all scenarios: " << simulated_steps << " steps, " << simulated_seconds << " s simulated in "
            << wall * 1000.0 << " ms (" << simulated_seconds / wall << "x real time)\n";
  return sink == 12345.678 ? 1 : 0;
}
