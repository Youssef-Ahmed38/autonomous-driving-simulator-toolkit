#include "autonomous_driving/bridge.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace autonomous_driving {
namespace {

constexpr std::size_t kMaxItems = 100000;

std::string error(const std::string& message) { return "error " + message; }

}  // namespace

std::string BridgeSession::handle_line(const std::string_view line) {
  std::istringstream in{std::string(line)};
  std::string command;
  if (!(in >> command)) return error("empty request");

  if (command == "route") {
    std::size_t count = 0;
    if (!(in >> count) || count == 0 || count > kMaxItems) return error("route needs a waypoint count in [1, 100000]");
    std::vector<Waypoint> route(count);
    for (auto& waypoint : route)
      if (!(in >> waypoint.position.x >> waypoint.position.y >> waypoint.target_speed)) return error("malformed route");
    route_ = std::move(route);
    return "ok route " + std::to_string(route_.size());
  }

  if (command == "frame") {
    if (route_.empty()) return error("send a route before frames");
    SensorFrame frame;
    std::size_t obstacle_count = 0;
    if (!(in >> frame.timestamp >> frame.gnss_position.x >> frame.gnss_position.y >> frame.gnss_sigma >> frame.speed >>
          frame.speed_sigma >> frame.heading_radians >> obstacle_count) || obstacle_count > kMaxItems)
      return error("malformed frame");
    frame.obstacles.resize(obstacle_count);
    for (auto& obstacle : frame.obstacles)
      if (!(in >> obstacle.position.x >> obstacle.position.y >> obstacle.velocity.x >> obstacle.velocity.y >>
            obstacle.radius >> obstacle.uncertainty))
        return error("malformed obstacle");
    if (frame.gnss_sigma <= 0.0 || frame.speed_sigma <= 0.0) return error("sigmas must be positive");

    const auto control = stack_.on_frame(frame, route_);
    const auto& estimate = stack_.estimate();
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "control " << control.steering << ' ' << control.acceleration << ' '
        << control.risk << ' ' << (std::isfinite(control.time_to_collision) ? control.time_to_collision : -1.0) << ' '
        << (control.emergency_brake ? 1 : 0) << ' ' << estimate.position.x << ' ' << estimate.position.y << ' '
        << estimate.uncertainty;
    return out.str();
  }

  if (command == "reset") {
    stack_.reset();
    return "ok reset";
  }

  if (command == "quit") {
    finished_ = true;
    return "bye";
  }

  return error("unknown command " + command);
}

}  // namespace autonomous_driving
