#pragma once

#include <cmath>
#include <numbers>

namespace research {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
  friend Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
  friend Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
  friend Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
  friend Vec2 operator*(Vec2 a, double scalar) { return {a.x * scalar, a.y * scalar}; }
  friend Vec2 operator*(double scalar, Vec2 a) { return a * scalar; }
  Vec2& operator+=(Vec2 other) { x += other.x; y += other.y; return *this; }
};

inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double norm(Vec2 value) { return std::sqrt(dot(value, value)); }
inline Vec2 normalized(Vec2 value) { const auto length = norm(value); return length > 1.0e-12 ? value * (1.0 / length) : Vec2{}; }
inline Vec2 from_heading(double heading_radians, double length = 1.0) { return {std::cos(heading_radians) * length, std::sin(heading_radians) * length}; }

/// Wraps an angle to (-pi, pi].
inline double wrap_angle(double radians) {
  constexpr double two_pi = 2.0 * std::numbers::pi;
  radians = std::fmod(radians + std::numbers::pi, two_pi);
  if (radians <= 0.0) radians += two_pi;
  return radians - std::numbers::pi;
}

}  // namespace research
