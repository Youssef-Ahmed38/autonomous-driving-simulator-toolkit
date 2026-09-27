#pragma once

#include "autonomous_driving/types.hpp"

namespace autonomous_driving {

struct VehicleLimits {
  double wheelbase = 2.8;       ///< meters
  double max_steering = 0.6;    ///< radians
  double max_acceleration = 3.0;
  double max_deceleration = 9.0;
  double max_speed = 40.0;
};

/// Rear-axle kinematic bicycle model. Speed is clamped to [0, max_speed]; the vehicle does not reverse.
class KinematicBicycleModel {
 public:
  explicit KinematicBicycleModel(VehicleLimits limits = {}) : limits_(limits) {}

  [[nodiscard]] VehicleState step(const VehicleState& state, const Control& control, double delta_seconds) const;
  [[nodiscard]] const VehicleLimits& limits() const { return limits_; }

 private:
  VehicleLimits limits_;
};

}  // namespace autonomous_driving
