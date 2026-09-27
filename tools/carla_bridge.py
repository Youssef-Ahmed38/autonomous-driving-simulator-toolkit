#!/usr/bin/env python3
"""Drive a CARLA ego vehicle with the C++ stack through `adsim bridge`.

The C++ process owns fusion and planning; this script only translates CARLA state into the
line protocol (see include/autonomous_driving/bridge.hpp) and applies the returned control.

    python tools/carla_bridge.py --adsim build/adsim --host localhost --port 2000

Requires the CARLA Python API (`pip install carla==<server version>`) and a running server.
Coordinates: CARLA/UE4 is left-handed (x forward, y right, yaw toward +y). The stack only
needs heading = atan2(vy, vx) and "positive steering increases heading", which holds in that
frame too, so positions and yaw are passed through unchanged and steer maps with a positive sign.
"""
from __future__ import annotations

import argparse
import math
import random
import subprocess
import sys

try:
    import carla
except ImportError:  # pragma: no cover - depends on the local CARLA install
    sys.exit("carla_bridge: the CARLA Python API is not installed (pip install carla==<server version>)")

NOMINAL_MAX_ACCEL = 3.0   # m/s^2 mapped to full throttle
NOMINAL_MAX_DECEL = 8.0   # m/s^2 mapped to full brake


def build_route(carla_map, start_location, spacing, count, speed):
    """Follows lane centre waypoints from the start; returns [(x, y, target_speed)]."""
    waypoint = carla_map.get_waypoint(start_location)
    route = []
    for _ in range(count):
        location = waypoint.transform.location
        route.append((location.x, location.y, speed))
        following = waypoint.next(spacing)
        if not following:
            break
        waypoint = following[0]
    if route:
        x, y, _ = route[-1]
        route[-1] = (x, y, 0.0)
    return route


def obstacle_radius(actor):
    extent = actor.bounding_box.extent
    return max(0.3, math.hypot(extent.x, extent.y))


def nearby_obstacles(world, ego, radius, sigma):
    ego_location = ego.get_location()
    obstacles = []
    for actor in world.get_actors():
        if actor.id == ego.id or not (actor.type_id.startswith("vehicle.") or actor.type_id.startswith("walker.")):
            continue
        location = actor.get_location()
        if location.distance(ego_location) > radius:
            continue
        velocity = actor.get_velocity()
        obstacles.append((location.x, location.y, velocity.x, velocity.y, obstacle_radius(actor), sigma))
    return obstacles


def frame_request(timestamp, ego, gnss_sigma, speed_sigma, obstacles, rng):
    transform = ego.get_transform()
    velocity = ego.get_velocity()
    gx = transform.location.x + rng.gauss(0.0, gnss_sigma)
    gy = transform.location.y + rng.gauss(0.0, gnss_sigma)
    speed = max(0.0, math.hypot(velocity.x, velocity.y) + rng.gauss(0.0, speed_sigma))
    heading = math.radians(transform.rotation.yaw)
    fields = [timestamp, gx, gy, gnss_sigma, speed, speed_sigma, heading, len(obstacles)]
    for obstacle in obstacles:
        fields.extend(obstacle)
    return "frame " + " ".join(f"{value:.6f}" if isinstance(value, float) else str(value) for value in fields)


def parse_control(line):
    parts = line.split()
    if not parts or parts[0] != "control" or len(parts) != 9:
        raise RuntimeError(f"unexpected bridge response: {line!r}")
    steering, acceleration, risk, ttc = (float(value) for value in parts[1:5])
    return steering, acceleration, risk, ttc, parts[5] == "1"


def to_vehicle_control(steering, acceleration, max_steer_radians):
    control = carla.VehicleControl()
    control.steer = max(-1.0, min(1.0, steering / max_steer_radians))
    if acceleration >= 0.0:
        control.throttle = min(1.0, acceleration / NOMINAL_MAX_ACCEL)
        control.brake = 0.0
    else:
        control.throttle = 0.0
        control.brake = min(1.0, -acceleration / NOMINAL_MAX_DECEL)
    return control


class Bridge:
    def __init__(self, executable):
        self.process = subprocess.Popen([executable, "bridge"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)

    def request(self, line):
        self.process.stdin.write(line + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip()
        if response.startswith("error"):
            raise RuntimeError(f"bridge rejected {line.split()[0]!r}: {response}")
        return response

    def close(self):
        try:
            self.request("quit")
        except (BrokenPipeError, RuntimeError, OSError):
            pass
        self.process.wait(timeout=5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--adsim", required=True, help="path to the adsim executable")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--vehicle", default="vehicle.tesla.model3")
    parser.add_argument("--spawn-index", type=int, default=0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=10.0, help="route target speed, m/s")
    parser.add_argument("--route-length", type=int, default=60, help="number of waypoints")
    parser.add_argument("--spacing", type=float, default=4.0, help="waypoint spacing, m")
    parser.add_argument("--gnss-sigma", type=float, default=0.5)
    parser.add_argument("--speed-sigma", type=float, default=0.1)
    parser.add_argument("--perception-sigma", type=float, default=0.3)
    parser.add_argument("--max-seconds", type=float, default=120.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)

    rng = random.Random(args.seed)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.dt
    world.apply_settings(settings)

    ego = None
    bridge = None
    try:
        blueprint = world.get_blueprint_library().filter(args.vehicle)[0]
        spawn_points = world.get_map().get_spawn_points()
        ego = world.spawn_actor(blueprint, spawn_points[args.spawn_index % len(spawn_points)])
        world.tick()
        max_steer = math.radians(max(wheel.max_steer_angle for wheel in ego.get_physics_control().wheels))

        route = build_route(world.get_map(), ego.get_location(), args.spacing, args.route_length, args.speed)
        bridge = Bridge(args.adsim)
        bridge.request("route " + str(len(route)) + " " + " ".join(f"{x:.3f} {y:.3f} {v:.3f}" for x, y, v in route))

        goal = carla.Location(x=route[-1][0], y=route[-1][1], z=ego.get_location().z)
        start_time = world.get_snapshot().timestamp.elapsed_seconds
        while True:
            world.tick()
            elapsed = world.get_snapshot().timestamp.elapsed_seconds - start_time
            obstacles = nearby_obstacles(world, ego, 60.0, args.perception_sigma)
            response = bridge.request(frame_request(elapsed, ego, args.gnss_sigma, args.speed_sigma, obstacles, rng))
            steering, acceleration, risk, ttc, emergency = parse_control(response)
            ego.apply_control(to_vehicle_control(steering, acceleration, max_steer))

            velocity = ego.get_velocity()
            speed = math.hypot(velocity.x, velocity.y)
            print(f"t={elapsed:6.2f}s speed={speed:5.2f} steer={steering:+.3f} accel={acceleration:+.2f} "
                  f"risk={risk:.2f} ttc={'-' if ttc < 0 else f'{ttc:.2f}'}{' EMERGENCY' if emergency else ''}")
            if ego.get_location().distance(goal) < 2.0 and speed < 0.5:
                print("goal reached")
                break
            if elapsed > args.max_seconds:
                print("time limit reached")
                break
    finally:
        if bridge is not None:
            bridge.close()
        if ego is not None:
            ego.destroy()
        world.apply_settings(original_settings)
    return 0


if __name__ == "__main__":
    sys.exit(main())
