#pragma once

#include "autonomous_driving/types.hpp"

#include <span>
#include <vector>

namespace autonomous_driving {

/// Constant-velocity prediction whose 1-sigma error grows with an unknown-acceleration term.
struct PredictedState {
  double time = 0.0;
  research::Vec2 position;
  double sigma = 0.0;
};

class ConstantVelocityPredictor {
 public:
  explicit ConstantVelocityPredictor(double acceleration_sigma = 1.0) : acceleration_sigma_(acceleration_sigma) {}

  [[nodiscard]] PredictedState at(research::Vec2 position, research::Vec2 velocity, double initial_sigma,
                                  double time) const;
  [[nodiscard]] std::vector<PredictedState> trajectory(const Obstacle& obstacle, double horizon_seconds,
                                                       double step_seconds) const;

 private:
  double acceleration_sigma_;
};

/// P(|X| < radius) for X ~ N(mean, sigma^2 I) in 2-D, i.e. the chance two uncertain discs overlap.
[[nodiscard]] double disc_collision_probability(double mean_distance, double combined_sigma, double radius);

/// Earliest t in [0, horizon] with |relative_position + relative_velocity * t| <= radius; infinity if none.
[[nodiscard]] double time_to_contact(research::Vec2 relative_position, research::Vec2 relative_velocity,
                                     double radius, double horizon_seconds);

struct RiskConfig {
  double ego_radius = 1.4;           ///< Disc enclosing the ego footprint, meters.
  double safety_margin = 0.5;        ///< Extra clearance added to the contact radius, meters.
  double step_seconds = 0.1;
  double acceleration_sigma = 0.3;   ///< Unknown obstacle acceleration driving prediction uncertainty growth, m/s^2.
};

struct ObstacleRisk {
  std::size_t index = 0;
  double collision_probability = 0.0;
  double time_of_peak = 0.0;
  double min_clearance = 0.0;        ///< Closest mean-trajectory gap between disc edges, meters.
  double time_to_collision = kNoCollision;
};

struct RiskAssessment {
  double risk = 0.0;
  double time_to_collision = kNoCollision;
  double min_clearance = kNoCollision;
  std::vector<ObstacleRisk> obstacles;
};

/// Rolls ego and obstacles forward under constant velocity and scores each pair by the peak
/// probability of overlap given both position uncertainties.
class RiskAssessor {
 public:
  explicit RiskAssessor(RiskConfig config = {}) : config_(config), predictor_(config.acceleration_sigma) {}

  [[nodiscard]] RiskAssessment assess(const VehicleState& ego, std::span<const Obstacle> obstacles,
                                      double horizon_seconds) const;
  [[nodiscard]] const RiskConfig& config() const { return config_; }

 private:
  RiskConfig config_;
  ConstantVelocityPredictor predictor_;
};

}  // namespace autonomous_driving
