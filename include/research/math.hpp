#pragma once

#include <cmath>

namespace research {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
  friend Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
  friend Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
  friend Vec2 operator*(Vec2 a, double scalar) { return {a.x * scalar, a.y * scalar}; }
};

inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec2 value) { return std::sqrt(dot(value, value)); }
inline Vec2 normalized(Vec2 value) { const auto length = norm(value); return length > 1.0e-12 ? value * (1.0 / length) : Vec2{}; }

}  // namespace research
