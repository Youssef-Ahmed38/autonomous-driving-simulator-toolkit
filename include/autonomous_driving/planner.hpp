#pragma once

#include "autonomous_driving/risk.hpp"
#include "autonomous_driving/types.hpp"

#include <cstddef>
#include <span>

namespace autonomous_driving {

struct PlannerConfig {
  double wheelbase = 2.8;
  double max_steering = 0.6;
  double lookahead_base = 4.0;       ///< meters
  double lookahead_gain = 0.6;       ///< seconds of travel added to the lookahead
  double max_lateral_acceleration = 3.0;
  double comfort_deceleration = 3.0;
  double max_deceleration = 8.0;
  double max_acceleration = 2.5;
  double speed_gain = 0.8;           ///< Proportional speed-tracking gain, 1/s.
  double risk_tolerance = 0.3;       ///< Collision probability tolerated before the target speed is reduced.
  double emergency_ttc = 1.5;        ///< Full braking below this time to collision, seconds.
  double goal_tolerance = 1.5;       ///< meters
  double following_time_gap = 1.5;   ///< IDM desired time headway to an in-lane lead, seconds.
  double standstill_gap = 2.0;       ///< IDM minimum bumper gap, meters.
  double lane_half_width = 1.8;      ///< Corridor half-width used to decide whether an obstacle is in path.
  RiskConfig risk;
};

struct RouteProgress {
  std::size_t segment = 0;           ///< Index i of the closest segment [i, i+1].
  research::Vec2 closest_point;
  double cross_track_error = 0.0;    ///< Signed: positive when the ego is left of the route.
  double distance_to_goal = 0.0;     ///< Along the route from the closest point.
  research::Vec2 lookahead_point;
  double reference_speed = 0.0;
  bool finished = false;
};

/// Pure-pursuit route follower with an uncertainty-aware longitudinal policy:
/// speed is capped by route targets, curvature, the stopping distance to the goal and the
/// collision risk; in-lane leads are followed with the Intelligent Driver Model; imminent
/// contacts trigger an emergency brake.
class RiskAwarePlanner {
 public:
  explicit RiskAwarePlanner(PlannerConfig config = {}) : config_(config), assessor_(config.risk) {}

  [[nodiscard]] Control plan(const VehicleState& ego, std::span<const Obstacle> obstacles,
                             std::span<const Waypoint> route, double horizon_seconds = 4.0) const;
  [[nodiscard]] RouteProgress progress(const VehicleState& ego, std::span<const Waypoint> route) const;
  [[nodiscard]] const PlannerConfig& config() const { return config_; }

 private:
  PlannerConfig config_;
  RiskAssessor assessor_;
};

}  // namespace autonomous_driving
