#pragma once

#include "research/math.hpp"

#include <limits>

namespace autonomous_driving {

inline constexpr double kNoCollision = std::numeric_limits<double>::infinity();

/// Ego estimate. `uncertainty` is the 1-sigma position error in meters.
struct VehicleState {
  research::Vec2 position;
  research::Vec2 velocity;
  double heading_radians = 0.0;
  double uncertainty = 0.0;
};

/// A tracked road user modelled as a disc. `uncertainty` is its 1-sigma position error in meters.
struct Obstacle {
  research::Vec2 position;
  research::Vec2 velocity;
  double radius = 1.0;
  double uncertainty = 0.0;
};

/// Actuation request. Steering is the front-wheel angle in radians (positive turns toward +heading),
/// acceleration is longitudinal in m/s^2 (negative brakes).
struct Control {
  double steering = 0.0;
  double acceleration = 0.0;
  double risk = 0.0;                        ///< Peak collision probability over the horizon, [0, 1].
  double time_to_collision = kNoCollision;  ///< Seconds until mean trajectories touch; infinity if never.
  bool emergency_brake = false;
};

struct Waypoint {
  research::Vec2 position;
  double target_speed = 0.0;  ///< m/s
};

}  // namespace autonomous_driving
