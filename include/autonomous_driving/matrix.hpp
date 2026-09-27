#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace autonomous_driving {

/// Fixed-size row-major matrix for the small linear algebra the filters need.
template <std::size_t Rows, std::size_t Cols>
struct Matrix {
  std::array<double, Rows * Cols> data{};

  double& operator()(std::size_t row, std::size_t col) { return data[row * Cols + col]; }
  double operator()(std::size_t row, std::size_t col) const { return data[row * Cols + col]; }

  static Matrix identity() {
    static_assert(Rows == Cols, "identity requires a square matrix");
    Matrix result;
    for (std::size_t i = 0; i < Rows; ++i) result(i, i) = 1.0;
    return result;
  }

  friend Matrix operator+(const Matrix& a, const Matrix& b) {
    Matrix result;
    for (std::size_t i = 0; i < Rows * Cols; ++i) result.data[i] = a.data[i] + b.data[i];
    return result;
  }

  friend Matrix operator-(const Matrix& a, const Matrix& b) {
    Matrix result;
    for (std::size_t i = 0; i < Rows * Cols; ++i) result.data[i] = a.data[i] - b.data[i];
    return result;
  }
};

template <std::size_t R, std::size_t K, std::size_t C>
Matrix<R, C> operator*(const Matrix<R, K>& a, const Matrix<K, C>& b) {
  Matrix<R, C> result;
  for (std::size_t r = 0; r < R; ++r)
    for (std::size_t k = 0; k < K; ++k) {
      const double value = a(r, k);
      if (value == 0.0) continue;
      for (std::size_t c = 0; c < C; ++c) result(r, c) += value * b(k, c);
    }
  return result;
}

template <std::size_t R, std::size_t C>
Matrix<C, R> transpose(const Matrix<R, C>& m) {
  Matrix<C, R> result;
  for (std::size_t r = 0; r < R; ++r)
    for (std::size_t c = 0; c < C; ++c) result(c, r) = m(r, c);
  return result;
}

inline Matrix<2, 2> inverse(const Matrix<2, 2>& m) {
  const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
  if (std::abs(det) < 1.0e-15) throw std::domain_error("singular 2x2 matrix");
  Matrix<2, 2> result;
  result(0, 0) = m(1, 1) / det;
  result(0, 1) = -m(0, 1) / det;
  result(1, 0) = -m(1, 0) / det;
  result(1, 1) = m(0, 0) / det;
  return result;
}

}  // namespace autonomous_driving
