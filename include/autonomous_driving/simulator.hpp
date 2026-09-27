#pragma once

#include "autonomous_driving/types.hpp"
#include "autonomous_driving/vehicle_model.hpp"

#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace autonomous_driving {

/// One tick of sensor data as a simulator (CARLA or the built-in one) would deliver it.
struct SensorFrame {
  double timestamp = 0.0;
  research::Vec2 gnss_position;
  double gnss_sigma = 0.5;
  double speed = 0.0;               ///< Wheel odometry, m/s.
  double speed_sigma = 0.1;
  double heading_radians = 0.0;     ///< IMU / compass yaw.
  std::vector<Obstacle> obstacles;  ///< Perception output in the world frame.
  bool collision = false;
};

/// Port implemented by every simulator backend. A CARLA adapter implements the same contract
/// (see docs/CARLA.md); the planning stack never depends on a concrete simulator.
class SimulatorAdapter {
 public:
  virtual ~SimulatorAdapter() = default;
  virtual SensorFrame reset() = 0;
  virtual SensorFrame step(const Control& control, double delta_seconds) = 0;
  [[nodiscard]] virtual std::vector<Waypoint> route() const = 0;
  [[nodiscard]] virtual bool done() const = 0;
  [[nodiscard]] virtual bool goal_reached() const = 0;
  /// Ground truth for evaluation, when the backend exposes it.
  [[nodiscard]] virtual std::optional<VehicleState> ground_truth() const { return std::nullopt; }
};

struct ScenarioActor {
  Obstacle initial;
  double stop_after_seconds = std::numeric_limits<double>::infinity();  ///< Actor halts abruptly at this time.
};

struct Scenario {
  std::string name;
  VehicleState ego_start;
  std::vector<Waypoint> route;
  std::vector<ScenarioActor> actors;
  double gnss_sigma = 0.5;
  double speed_sigma = 0.1;
  double perception_sigma = 0.3;    ///< Noise on reported obstacle positions.
  double time_limit_seconds = 60.0;
  unsigned seed = 7;
};

namespace scenarios {
Scenario straight_road();
Scenario curved_road();
Scenario pedestrian_crossing();
Scenario lead_vehicle_braking();
std::vector<Scenario> all();
}  // namespace scenarios

/// Deterministic in-process simulator: kinematic bicycle ego, constant-velocity actors,
/// Gaussian sensor noise and disc-overlap collision detection.
class KinematicSimulator final : public SimulatorAdapter {
 public:
  explicit KinematicSimulator(Scenario scenario, VehicleLimits limits = {}, double ego_radius = 1.4);

  SensorFrame reset() override;
  SensorFrame step(const Control& control, double delta_seconds) override;
  [[nodiscard]] std::vector<Waypoint> route() const override { return scenario_.route; }
  [[nodiscard]] bool done() const override;
  [[nodiscard]] bool goal_reached() const override;
  [[nodiscard]] std::optional<VehicleState> ground_truth() const override { return ego_; }

  [[nodiscard]] double time() const { return time_; }
  [[nodiscard]] bool collided() const { return collided_; }
  [[nodiscard]] double min_clearance() const { return min_clearance_; }
  [[nodiscard]] const Scenario& scenario() const { return scenario_; }
  [[nodiscard]] std::vector<Obstacle> actors_truth() const;

 private:
  SensorFrame observe();
  void check_collisions();

  Scenario scenario_;
  KinematicBicycleModel model_;
  double ego_radius_;
  VehicleState ego_{};
  std::vector<Obstacle> actors_;
  double time_ = 0.0;
  bool collided_ = false;
  double min_clearance_ = std::numeric_limits<double>::infinity();
  std::mt19937 rng_;
};

}  // namespace autonomous_driving
