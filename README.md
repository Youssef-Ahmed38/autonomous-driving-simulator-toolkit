# Autonomous Driving Simulator Toolkit

C++20 uncertainty-aware sensor fusion, risk prediction and planning core designed to sit behind a
simulator adapter. It ships with a deterministic in-process kinematic simulator for tests and
demos, and a stdin/stdout bridge plus a Python client so CARLA can drive the same stack.

## What's inside

| Module | Header | What it does |
|---|---|---|
| Vehicle model | `vehicle_model.hpp` | Rear-axle kinematic bicycle model with steering, acceleration and speed limits |
| Sensor fusion | `sensor_fusion.hpp` | Constant-velocity Kalman filter (GNSS position + odometry velocity, commanded-acceleration input, chi-square gating with lockout recovery); lightweight alpha-beta tracker with propagated uncertainty |
| Risk prediction | `risk.hpp` | Constant-velocity prediction with growing uncertainty, exact time-to-contact, probability of disc overlap for Gaussian position error (Rice integral), per-obstacle risk report |
| Planning | `planner.hpp` | Route progress on a polyline, pure-pursuit steering, speed capped by route targets / curvature / goal stopping distance / collision risk, IDM car following, TTC emergency braking |
| Simulation port | `simulator.hpp` | `SimulatorAdapter` interface, `KinematicSimulator` with sensor noise and collision detection, four built-in scenarios |
| Stack | `driving_stack.hpp` | Sensor frame in, control out; `run_episode` closed-loop driver with metrics |
| Bridge | `bridge.hpp` | Line protocol so an out-of-process simulator client can drive the stack |

`autonomous_driving/driving.hpp` includes everything.

## Build and run

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/adsim demo all
./build/adsim_benchmark
```

On Visual Studio generators the binaries are under `build/Release/`.

```text
adsim list                                  list built-in scenarios
adsim demo [scenario|all] [--dt S] [--csv trace.csv]
adsim bridge                                serve the line protocol on stdin/stdout
```

`demo` exits with 2 if any scenario ends in a collision. `--csv` writes a per-step trace (truth,
estimate, control, risk, TTC) for plotting.

## Built-in scenarios

| Scenario | Checks | Expected result |
|---|---|---|
| `straight_road` | speed tracking, stopping at the goal | completes |
| `curved_road` | pure pursuit through a 25 m radius quarter turn, curvature speed limit | completes |
| `pedestrian_crossing` | a pedestrian crosses the lane ahead; risk-based slowdown | completes; slows to ~4 m/s while the pedestrian crosses, then resumes ~11 m/s |
| `lead_vehicle_braking` | lead car brakes to a standstill; IDM following + TTC braking | stops behind it with a gap |

All four run collision-free with a mean fused position error of about 0.1 m against 0.5 m GNSS noise.

## Using it with CARLA

The planning stack depends only on `SimulatorAdapter` / `SensorFrame`. For CARLA:

- `tools/carla_bridge.py` spawns an ego vehicle, builds a route from lane waypoints, and forwards
  pose, speed and nearby actors to `adsim bridge`. It then applies the returned steering and
  throttle/brake.
- For an in-process C++ integration, implement `SimulatorAdapter` against LibCarla instead.

See [docs/CARLA.md](docs/CARLA.md) for the protocol, coordinate conventions and both options.

## Design notes

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the data flow, the models and their
limitations.
