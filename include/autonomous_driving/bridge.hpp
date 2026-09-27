#pragma once

#include "autonomous_driving/driving_stack.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace autonomous_driving {

/// Line-oriented protocol that lets an out-of-process simulator client (for example the CARLA
/// Python API, see tools/carla_bridge.py) drive the stack over stdin/stdout.
///
/// Requests (whitespace separated, one per line):
///   route <n> {x y target_speed}*n
///   frame <t> <gx> <gy> <gnss_sigma> <speed> <speed_sigma> <heading> <m> {x y vx vy radius sigma}*m
///   reset
///   quit
/// Responses:
///   ok route <n>
///   control <steering> <acceleration> <risk> <ttc|-1> <emergency 0|1> <est_x> <est_y> <est_sigma>
///   ok reset
///   bye
///   error <message>
class BridgeSession {
 public:
  explicit BridgeSession(StackConfig config = {}) : stack_(config) {}

  /// Handles one request line and returns the response line (without a trailing newline).
  std::string handle_line(std::string_view line);
  [[nodiscard]] bool finished() const { return finished_; }
  [[nodiscard]] const DrivingStack& stack() const { return stack_; }

 private:
  DrivingStack stack_;
  std::vector<Waypoint> route_;
  bool finished_ = false;
};

}  // namespace autonomous_driving
