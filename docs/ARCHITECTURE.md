# Architecture

```text
 CARLA (tools/carla_bridge.py) ──stdin/stdout──► BridgeSession ─┐
 KinematicSimulator ─────────── SimulatorAdapter ────────────────┤
                                                                 ▼
                                     SensorFrame ──► DrivingStack
                                                       │
                          ConstantVelocityKalmanFilter │  GNSS + odometry → ego estimate (+ sigma)
                                                       ▼
                                                RiskAwarePlanner
                                      ┌────────────────┼──────────────────┐
                              route progress     RiskAssessor         IDM lead
                              + pure pursuit   (prediction, TTC,     following
                                               overlap probability)
                                                       ▼
                                                    Control ──► simulator
```

Dependencies point inward. The planner and fusion know nothing about CARLA, the kinematic
simulator, or the bridge.

## Sensor fusion

`ConstantVelocityKalmanFilter` state is `[px, py, vx, vy]`, with a discrete white-noise
acceleration process model.

- **Prediction** includes the previously commanded acceleration along the heading. Without it,
  hard braking looks like a model violation.
- **Position updates** come from GNSS. **Velocity updates** come from wheel speed plus IMU heading.
  The velocity sigma grows with speed to account for heading noise.
- **Gating** uses the normalized innovation squared against a chi-square (2 dof) 99.9 % threshold.
  Each measurement type keeps its own rejection counter. After 5 consecutive rejections, the filter
  assumes it has diverged: it drops correlations, re-opens the variance of the measured states, and
  accepts the measurement. This is what keeps a relocalization jump or an unmodelled manoeuvre from
  locking the filter out.
- **Updates** use the Joseph form to keep the covariance symmetric positive semi-definite.

`AlphaBetaSensorFusion` is kept as a cheap baseline. Its gains shrink with measurement noise, and
the estimate's sigma is propagated as a convex blend of the prediction and the measurement.

## Risk

Each obstacle is a disc with Gaussian position error. Ego and obstacles are rolled forward at
constant velocity over the horizon (default 4 s, 0.1 s steps).

- The obstacle's sigma grows as `hypot(sigma0, 0.5 * a_sigma * t^2)`. The ego's own estimate error
  stays constant, since the ego follows its plan.
- At each step, the probability that the relative position lies inside the contact radius
  (body radii + margin) is the integral of a Rice density. It is evaluated with Simpson's rule
  over a ±8 sigma window, using a polynomial approximation of the scaled Bessel I0. Tests check it
  against the closed form at zero offset and against Monte Carlo off-centre.
- Obstacle risk is the peak probability over the horizon. Scene risk is the maximum over obstacles.
- Time to contact is solved analytically from the relative motion quadratic.

Known limitation: isotropic uncertainty and constant-velocity prediction. Larger uncertainty
dilutes the peak probability of a direct hit, which is why TTC braking and IDM run alongside the
probability term rather than being replaced by it.

## Planning

1. **Route progress.** Find the closest point over all polyline segments. This is stateless, so it
   tolerates dropped frames, at the cost of ambiguity on self-intersecting routes. It yields the
   segment, signed cross-track error, distance to goal, and interpolated reference speed.
2. **Steering.** Pure pursuit toward a point `lookahead_base + lookahead_gain * speed` ahead along
   the route: `delta = atan(L * 2 sin(alpha) / d)`, clamped.
3. **Speed target.** The minimum of the route speed, the curvature limit `sqrt(a_lat / |kappa|)`,
   and the goal stopping speed `sqrt(2 * a_comfort * room)`. It is then scaled down linearly once
   the risk exceeds `risk_tolerance`.
4. **Acceleration.** A proportional speed tracker, then capped by:
   - goal feed-forward braking (`v^2 / 2d` once that reaches half the comfort deceleration),
   - the IDM interaction term for the nearest obstacle inside the lane corridor ahead,
   - the TTC stopping requirement,
   - full braking when TTC is below `emergency_ttc`.

## Simulator

`KinematicSimulator` integrates the ego with the bicycle model (midpoint heading integration) and
moves actors at constant velocity, optionally stopping them abruptly at a set time. It samples
GNSS, speed, heading and perception noise from a seeded `std::mt19937`, and detects collisions as
disc overlap. Runs are deterministic for a given standard library. The distributions are not
bit-identical across standard library implementations, so tests assert behaviour, not exact
trajectories.

## Complexity

| Operation | Cost |
|---|---|
| Route progress | O(W) in waypoints |
| Risk assessment | O(N · H/Δt · 128) in obstacles; ~50 µs for 32 obstacles and a 4 s horizon |
| Kalman predict + 2 updates | fixed 4×4 algebra, ~0.2 µs |
