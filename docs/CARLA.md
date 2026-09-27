# CARLA integration

There are two ways to put the stack behind CARLA.

## Option A: out-of-process bridge (included)

`adsim bridge` reads requests on stdin and writes one response per line on stdout.
`tools/carla_bridge.py` is the CARLA-side client.

```bash
pip install carla==<your server version>
./CarlaUE4.sh            # or CarlaUE4.exe on Windows
python tools/carla_bridge.py --adsim build/adsim --spawn-index 3 --speed 10
```

The script puts the world in synchronous mode (`--dt`, default 0.05 s) and spawns
`vehicle.tesla.model3`. It builds a route by following lane-centre waypoints from the spawn
point, then runs this loop on every tick:

1. Sends the ego pose (with synthetic GNSS noise), speed, yaw, and all vehicles and walkers
   within 60 m.
2. Maps the returned control onto `carla.VehicleControl`:
   - `steer = steering / max_steer_angle`
   - `throttle = a / 3`
   - `brake = -a / 8`

On exit it destroys the ego and restores the original world settings.

Verification status: the script has been run end to end against the real `adsim` binary, using a
minimal fake `carla` module (a straight road with a crossing walker). It reached the goal, slowed
for the walker, and cleaned up. It has **not yet been run against a live CARLA server**, so expect
to tune the throttle/brake mapping and the actor radii for your CARLA version and vehicle.

### Protocol

Requests:

```text
route <n> {x y target_speed}*n
frame <t> <gx> <gy> <gnss_sigma> <speed> <speed_sigma> <heading> <m> {x y vx vy radius sigma}*m
reset
quit
```

Responses:

```text
ok route <n>
control <steering> <acceleration> <risk> <ttc|-1> <emergency 0|1> <est_x> <est_y> <est_sigma>
ok reset
bye
error <message>
```

Units: meters, seconds, radians, m/s, m/s². `ttc` is `-1` when no contact is predicted. A frame
sent before a route, a malformed request, or a non-positive sigma returns `error ...`, and the
session stays usable afterwards.

### Coordinate conventions

CARLA/UE4 is left-handed: x forward, y right, and yaw rotates from +x toward +y. The stack only
assumes two things:

- `heading = atan2(vy, vx)`
- positive steering increases heading

Both hold in CARLA's frame. So locations and `radians(yaw)` pass through unchanged, and a positive
steering output maps to a positive (right) CARLA steer.

## Option B: in-process adapter

Implement `autonomous_driving::SimulatorAdapter` against LibCarla (the CARLA C++ client):

| Method | CARLA source |
|---|---|
| `reset()` | spawn or teleport the ego, tick once, return the first `SensorFrame` |
| `step(control, dt)` | `ApplyControl`, `World::Tick`, then read the ego transform and velocity, GNSS/IMU sensor callbacks, and nearby actors |
| `route()` | lane waypoints (`Map::GetWaypoint`, `Waypoint::GetNext`) |
| `done()` / `goal_reached()` | distance to the last waypoint, a collision sensor, a time limit |
| `ground_truth()` | the ego transform and velocity, for evaluation metrics |

`run_episode` then drives CARLA exactly the way it drives `KinematicSimulator`. This option is
not included, because LibCarla has to be built against a specific server version.
