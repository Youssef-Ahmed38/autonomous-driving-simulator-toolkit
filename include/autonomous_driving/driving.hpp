#pragma once

#include "research/math.hpp"

#include <span>
#include <vector>

namespace autonomous_driving {

struct VehicleState { research::Vec2 position; research::Vec2 velocity; double heading_radians = 0.0; double uncertainty = 0.0; };
struct Obstacle { research::Vec2 position; research::Vec2 velocity; double radius = 1.0; double uncertainty = 0.0; };
struct Control { double steering = 0.0; double acceleration = 0.0; double risk = 0.0; };
struct Waypoint { research::Vec2 position; double target_speed = 0.0; };

class RiskAwarePlanner {
 public:
  [[nodiscard]] Control plan(const VehicleState& ego, std::span<const Obstacle> obstacles,
                             std::span<const Waypoint> route, double horizon_seconds = 4.0) const;
};

class AlphaBetaSensorFusion {
 public:
  AlphaBetaSensorFusion(double alpha = 0.75, double beta = 0.20) : alpha_(alpha), beta_(beta) {}
  [[nodiscard]] VehicleState update(research::Vec2 measured_position, double measurement_uncertainty,
                                    double delta_seconds);
 private:
  double alpha_; double beta_; bool initialized_ = false; VehicleState estimate_{};
};

}  // namespace autonomous_driving
