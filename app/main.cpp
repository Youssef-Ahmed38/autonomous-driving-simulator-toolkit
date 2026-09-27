#include "autonomous_driving/driving.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kUsage =
    "usage:\n"
    "  adsim list                                  list built-in scenarios\n"
    "  adsim demo [scenario|all] [--dt S] [--csv trace.csv]\n"
    "                                              run closed-loop scenarios on the kinematic simulator\n"
    "  adsim bridge                                serve the line protocol on stdin/stdout (see docs/CARLA.md)";

const char* status_of(const autonomous_driving::EpisodeResult& result) {
  if (result.collided) return "collision";
  if (result.completed) return "completed";
  return "stopped";
}

int run_demo(const std::string& which, const double dt, const std::string& csv_path) {
  using namespace autonomous_driving;
  std::vector<Scenario> selected;
  for (auto& scenario : scenarios::all())
    if (which == "all" || scenario.name == which) selected.push_back(std::move(scenario));
  if (selected.empty()) throw std::invalid_argument("unknown scenario: " + which);

  std::ofstream csv;
  if (!csv_path.empty()) {
    csv.open(csv_path);
    if (!csv) throw std::runtime_error("cannot write " + csv_path);
    csv << "scenario,time,true_x,true_y,est_x,est_y,est_sigma,speed,steering,acceleration,risk,ttc,emergency\n";
    csv << std::fixed << std::setprecision(4);
  }

  bool any_collision = false;
  for (const auto& scenario : selected) {
    KinematicSimulator simulator(scenario);
    DrivingStack stack;
    const auto result = run_episode(simulator, stack, dt, scenario.time_limit_seconds, [&](const StepRecord& step) {
      if (!csv.is_open()) return;
      const auto truth = step.truth.value_or(VehicleState{});
      csv << scenario.name << ',' << step.time << ',' << truth.position.x << ',' << truth.position.y << ','
          << step.estimate.position.x << ',' << step.estimate.position.y << ',' << step.estimate.uncertainty << ','
          << research::norm(truth.velocity) << ',' << step.control.steering << ',' << step.control.acceleration << ','
          << step.control.risk << ',' << (std::isfinite(step.control.time_to_collision) ? step.control.time_to_collision : -1.0)
          << ',' << (step.control.emergency_brake ? 1 : 0) << '\n';
    });
    any_collision = any_collision || result.collided;
    std::cout << std::fixed << std::setprecision(2) << std::left << std::setw(22) << scenario.name
              << " status=" << std::setw(9) << status_of(result) << " time=" << result.duration_seconds
              << "s min_clearance=" << simulator.min_clearance() << "m max_risk=" << result.max_risk
              << " emergency_steps=" << result.emergency_brakes << " pos_err_mean=" << result.mean_position_error
              << "m pos_err_max=" << result.max_position_error << "m\n";
  }
  return any_collision ? 2 : 0;
}

int run_bridge() {
  autonomous_driving::BridgeSession session;
  std::string line;
  while (!session.finished() && std::getline(std::cin, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::cout << session.handle_line(line) << '\n' << std::flush;
  }
  return 0;
}

}  // namespace

int main(const int argc, char** argv) {
  try {
    if (argc < 2) throw std::invalid_argument(kUsage);
    const std::string mode = argv[1];
    if (mode == "list") {
      for (const auto& scenario : autonomous_driving::scenarios::all()) std::cout << scenario.name << '\n';
      return 0;
    }
    if (mode == "bridge") return run_bridge();
    if (mode == "demo") {
      std::string which = "all";
      std::string csv_path;
      double dt = 0.05;
      for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--dt" && index + 1 < argc) dt = std::stod(argv[++index]);
        else if (option == "--csv" && index + 1 < argc) csv_path = argv[++index];
        else if (!option.starts_with("--")) which = option;
        else throw std::invalid_argument("unknown or incomplete option: " + option);
      }
      if (!(dt > 0.0 && dt <= 0.5)) throw std::invalid_argument("--dt must be in (0, 0.5]");
      return run_demo(which, dt, csv_path);
    }
    throw std::invalid_argument(kUsage);
  } catch (const std::exception& error) {
    std::cerr << "adsim: " << error.what() << '\n';
    return 1;
  }
}
