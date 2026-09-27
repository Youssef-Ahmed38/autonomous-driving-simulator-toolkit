#pragma once

#include "autonomous_driving/planner.hpp"
#include "autonomous_driving/sensor_fusion.hpp"
#include "autonomous_driving/simulator.hpp"

#include <functional>
#include <span>

namespace autonomous_driving {

struct StackConfig {
  PlannerConfig planner;
  KalmanConfig fusion;
  double horizon_seconds = 4.0;
};

/// Sensor frame in, control out: Kalman fusion of GNSS + odometry, then risk-aware planning.
class DrivingStack {
 public:
  explicit DrivingStack(StackConfig config = {}) : config_(config), filter_(config.fusion), planner_(config.planner) {}

  Control on_frame(const SensorFrame& frame, std::span<const Waypoint> route);
  void reset();

  [[nodiscard]] const VehicleState& estimate() const { return estimate_; }
  [[nodiscard]] const ConstantVelocityKalmanFilter& filter() const { return filter_; }
  [[nodiscard]] const RiskAwarePlanner& planner() const { return planner_; }
  [[nodiscard]] std::size_t rejected_measurements() const { return rejected_; }

 private:
  StackConfig config_;
  ConstantVelocityKalmanFilter filter_;
  RiskAwarePlanner planner_;
  VehicleState estimate_{};
  double last_timestamp_ = 0.0;
  double last_acceleration_ = 0.0;
  std::size_t rejected_ = 0;
};

struct StepRecord {
  double time = 0.0;
  const SensorFrame* frame = nullptr;
  VehicleState estimate;
  std::optional<VehicleState> truth;
  Control control;
};

struct EpisodeResult {
  std::size_t steps = 0;
  double duration_seconds = 0.0;
  bool collided = false;
  bool completed = false;          ///< Reached the end of the route (adapter reported done without collision).
  double max_risk = 0.0;
  double mean_position_error = 0.0;  ///< Only when the adapter exposes ground truth.
  double max_position_error = 0.0;
  std::size_t emergency_brakes = 0;
};

/// Closed-loop driver shared by the demo, tests and benchmarks.
EpisodeResult run_episode(SimulatorAdapter& simulator, DrivingStack& stack, double delta_seconds,
                          double time_limit_seconds, const std::function<void(const StepRecord&)>& on_step = {});

}  // namespace autonomous_driving
