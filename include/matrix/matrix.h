// Matrix types and linear algebra helpers for the sensor fusion core.
//
// Matrix<T, Rows, Cols> has its sizes in the type and stores its elements in
// a std::array. Matrix<T, 0, 0> has its sizes set at run time and stores its
// elements in a std::vector. Both share the same free functions.

#ifndef SENSOR_FUSION_MATRIX_MATRIX_H_
#define SENSOR_FUSION_MATRIX_MATRIX_H_

#include "matrix/simd.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(__clang__)
#define MATRIX_SCALAR_FOR \
  _Pragma("clang loop vectorize(disable) interleave(disable)")
#elif defined(__GNUC__) && (__GNUC__ >= 14)
#define MATRIX_SCALAR_FOR _Pragma("GCC novector")
#else
#define MATRIX_SCALAR_FOR
#endif

// Matrix with sizes fixed at compile time.
template <typename T, std::size_t Rows = 0, std::size_t Cols = Rows>
class Matrix {
 public:
  Matrix() {
    for (T& element : data_) {
      element = T();
    }
  }

  static constexpr std::size_t rows() { return Rows; }
  static constexpr std::size_t cols() { return Cols; }

  T& operator()(std::size_t row, std::size_t col) {
    return data_[row * Cols + col];
  }
  const T& operator()(std::size_t row, std::size_t col) const {
    return data_[row * Cols + col];
  }

  T* data() noexcept { return data_.data(); }
  const T* data() const noexcept { return data_.data(); }

 private:
  std::array<T, Rows * Cols> data_;
};

// Matrix with sizes set at run time.
template <typename T>
class Matrix<T, 0, 0> {
 public:
  Matrix(std::size_t rows, std::size_t cols)
      : rows_(rows), cols_(cols), data_(rows * cols) {}

  std::size_t rows() const { return rows_; }
  std::size_t cols() const { return cols_; }

  T& operator()(std::size_t row, std::size_t col) {
    return data_[row * cols_ + col];
  }

  const T& operator()(std::size_t row, std::size_t col) const {
    return data_[row * cols_ + col];
  }

  T* data() noexcept { return data_.data(); }
  const T* data() const noexcept { return data_.data(); }

 private:
  std::size_t rows_;
  std::size_t cols_;
  std::vector<T> data_;
};

// Vector: a matrix with one column.
template <typename T, std::size_t N>
using Vector = Matrix<T, N, 1>;

// Square matrix with N rows and N columns.
template <typename T, std::size_t N>
using SquareMatrix = Matrix<T, N, N>;

// Square matrix with a size set at run time. It converts to Matrix<T, 0, 0>.
template <typename T>
class DynamicSquareMatrix {
 public:
  explicit DynamicSquareMatrix(std::size_t n)
      : matrix_(require_size(n), require_size(n)) {}

  std::size_t rows() const { return matrix_.rows(); }
  std::size_t cols() const { return matrix_.cols(); }

  T& operator()(std::size_t row, std::size_t col) {
    return matrix_(row, col);
  }
  const T& operator()(std::size_t row, std::size_t col) const {
    return matrix_(row, col);
  }

  T* data() noexcept { return matrix_.data(); }
  const T* data() const noexcept { return matrix_.data(); }

  operator const Matrix<T, 0, 0>&() const { return matrix_; }

 private:
  static std::size_t require_size(std::size_t n) {
    if (n == 0) {
      throw std::invalid_argument("Square matrix size must be positive");
    }
    return n;
  }

  Matrix<T, 0, 0> matrix_;
};

template <typename T>
T zero_element() {
  return T();
}

// True when value is negligible relative to scale. For floating point types
// the tolerance is machine epsilon times scale. Other types compare with zero.
template <typename T>
bool is_zero(const T& value, const T& scale) {
  if constexpr (std::is_floating_point<T>::value) {
    using std::abs;
    return abs(value) <= std::numeric_limits<T>::epsilon() * abs(scale);
  } else {
    return value == zero_element<T>();
  }
}

// Largest absolute value in one column.
template <typename T, std::size_t Rows, std::size_t Cols>
T column_scale(const Matrix<T, Rows, Cols>& a, std::size_t column) {
  using std::abs;
  T scale = zero_element<T>();
  for (std::size_t row = 0; row < a.rows(); ++row) {
    const T magnitude = abs(a(row, column));
    if (magnitude > scale) {
      scale = magnitude;
    }
  }
  return scale;
}

template <typename T>
T one_element() {
  if constexpr (std::is_floating_point<T>::value ||
                std::is_integral<T>::value) {
    return static_cast<T>(1);
  } else {
    // For ratio types.
    return T(1, 1);
  }
}

// Adding two matrices.
template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols>& a,
                                const Matrix<T, Rows, Cols>& b) {
  if (a.cols() != b.cols() || a.rows() != b.rows()) {
    throw std::invalid_argument("Matrix sizes differ!");
  }

  Matrix<T, Rows, Cols> result = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<T, Rows, Cols>(a.rows(), a.cols());
    } else {
      return Matrix<T, Rows, Cols>{};
    }
  }();

  const std::size_t count = a.rows() * a.cols();
  if constexpr (std::is_same<T, double>::value) {
    if (MATRIX_USE_SIMD && simd::enabled()) {
      simd::add(result.data(), a.data(), b.data(), count);
      return result;
    }
  }
  MATRIX_SCALAR_FOR
  for (std::size_t r = 0; r < a.rows(); ++r) {
    MATRIX_SCALAR_FOR
    for (std::size_t c = 0; c < a.cols(); ++c) {
      result(r, c) = a(r, c) + b(r, c);
    }
  }
  return result;
}

// Subtracting matrix b from matrix a.
template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator-(const Matrix<T, Rows, Cols>& a,
                                const Matrix<T, Rows, Cols>& b) {
  if (a.cols() != b.cols() || a.rows() != b.rows()) {
    throw std::invalid_argument("Matrix sizes differ!");
  }

  Matrix<T, Rows, Cols> result = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<T, Rows, Cols>(a.rows(), a.cols());
    } else {
      return Matrix<T, Rows, Cols>{};
    }
  }();

  const std::size_t count = a.rows() * a.cols();
  if constexpr (std::is_same<T, double>::value) {
    if (MATRIX_USE_SIMD && simd::enabled()) {
      simd::sub(result.data(), a.data(), b.data(), count);
      return result;
    }
  }
  MATRIX_SCALAR_FOR
  for (std::size_t r = 0; r < a.rows(); ++r) {
    MATRIX_SCALAR_FOR
    for (std::size_t c = 0; c < a.cols(); ++c) {
      result(r, c) = a(r, c) - b(r, c);
    }
  }
  return result;
}

// Multiplication by a scalar.
template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const Matrix<T, Rows, Cols>& a, const T k) {
  Matrix<T, Rows, Cols> result = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<T, Rows, Cols>(a.rows(), a.cols());
    } else {
      return Matrix<T, Rows, Cols>{};
    }
  }();

  const std::size_t count = a.rows() * a.cols();
  if constexpr (std::is_same<T, double>::value) {
    if (MATRIX_USE_SIMD && simd::enabled()) {
      simd::scale(result.data(), a.data(), k, count);
      return result;
    }
  }
  MATRIX_SCALAR_FOR
  for (std::size_t r = 0; r < a.rows(); ++r) {
    MATRIX_SCALAR_FOR
    for (std::size_t c = 0; c < a.cols(); ++c) {
      result(r, c) = k * a(r, c);
    }
  }
  return result;
}

template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const T k, const Matrix<T, Rows, Cols>& a) {
  return a * k;
}

// Multiplication of two matrices.
template <typename T, std::size_t Rows, std::size_t Inner, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const Matrix<T, Rows, Inner>& a,
                                const Matrix<T, Inner, Cols>& b) {
  if (a.cols() != b.rows()) {
    throw std::invalid_argument("Incorrect matrix sizes!");
  }

  Matrix<T, Rows, Cols> result = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<T, Rows, Cols>(a.rows(), b.cols());
    } else {
      return Matrix<T, Rows, Cols>{};
    }
  }();

  if constexpr (std::is_same<T, double>::value) {
    if (MATRIX_USE_SIMD && simd::enabled()) {
      const std::size_t cols = b.cols();
      for (std::size_t row = 0; row < a.rows(); ++row) {
        double* destination = result.data() + row * cols;
        for (std::size_t k = 0; k < a.cols(); ++k) {
          simd::axpy(destination, b.data() + k * cols, a(row, k), cols);
        }
      }
      return result;
    }
  }
  MATRIX_SCALAR_FOR
  for (std::size_t fixed_row = 0; fixed_row < a.rows(); ++fixed_row) {
    MATRIX_SCALAR_FOR
    for (std::size_t fixed_col = 0; fixed_col < b.cols(); ++fixed_col) {
      T sum = zero_element<T>();
      MATRIX_SCALAR_FOR
      for (std::size_t k = 0; k < a.cols(); ++k) {
        sum += a(fixed_row, k) * b(k, fixed_col);
      }
      result(fixed_row, fixed_col) = sum;
    }
  }
  return result;
}

// Converts every element to double.
template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<double, Rows, Cols> as_double(const Matrix<T, Rows, Cols>& a) {
  Matrix<double, Rows, Cols> converted = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<double, Rows, Cols>(a.rows(), a.cols());
    } else {
      return Matrix<double, Rows, Cols>{};
    }
  }();
  for (std::size_t row = 0; row < a.rows(); ++row) {
    for (std::size_t col = 0; col < a.cols(); ++col) {
      converted(row, col) = static_cast<double>(a(row, col));
    }
  }
  return converted;
}

// Determinant, by Gaussian elimination with partial pivoting.
template <typename T, std::size_t Rows, std::size_t Cols>
auto det(const Matrix<T, Rows, Cols>& a) {
  if (a.cols() != a.rows()) {
    throw std::invalid_argument(
        "Incorrect matrix size: matrix has to be square!");
  }

  if constexpr (!std::is_floating_point<T>::value) {
    return det(as_double(a));
  } else {
    Matrix<T, Rows, Cols> m = a;
    const std::size_t n = m.rows();
    T result = one_element<T>();

    using std::abs;

    for (std::size_t k = 0; k < n; ++k) {
      std::size_t pivot = k;
      for (std::size_t row = k + 1; row < n; ++row) {
        if (abs(m(row, k)) > abs(m(pivot, k))) {
          pivot = row;
        }
      }

      if (is_zero(m(pivot, k), column_scale(a, k))) {
        return zero_element<T>();
      }

      if (pivot != k) {
        for (std::size_t col = 0; col < n; ++col) {
          std::swap(m(k, col), m(pivot, col));
        }
        result = -result;
      }

      for (std::size_t row = k + 1; row < n; ++row) {
        const T factor = m(row, k) / m(k, k);
        bool used_simd = false;
        if constexpr (std::is_same<T, double>::value) {
          if (MATRIX_USE_SIMD && simd::enabled()) {
            simd::axpy(m.data() + row * n + k, m.data() + k * n + k, -factor,
                       n - k);
            used_simd = true;
          }
        }
        if (!used_simd) {
          MATRIX_SCALAR_FOR
          for (std::size_t col = k; col < n; ++col) {
            m(row, col) -= factor * m(k, col);
          }
        }
      }
    }

    for (std::size_t i = 0; i < n; ++i) {
      result *= m(i, i);
    }

    return result;
  }
}

// Matrix inverse, by Gauss-Jordan elimination with partial pivoting.
template <typename T, std::size_t Rows, std::size_t Cols>
auto inverse(const Matrix<T, Rows, Cols>& a) {
  if (a.rows() != a.cols()) {
    throw std::invalid_argument(
        "Incorrect matrix size: matrix has to be square!");
  }

  if constexpr (!std::is_floating_point<T>::value) {
    return inverse(as_double(a));
  } else {
    const std::size_t n = a.rows();

    Matrix<T, Rows, Cols * 2> augmented = [&] {
      if constexpr (Rows == 0 && Cols == 0) {
        return Matrix<T, Rows, Cols * 2>(n, n * 2);
      } else {
        return Matrix<T, Rows, Cols * 2>{};
      }
    }();

    for (std::size_t row = 0; row < n; ++row) {
      for (std::size_t col = 0; col < n; ++col) {
        augmented(row, col) = a(row, col);
      }
      augmented(row, n + row) = one_element<T>();
    }

    using std::abs;

    for (std::size_t k = 0; k < n; ++k) {
      std::size_t pivot = k;
      for (std::size_t row = k + 1; row < n; ++row) {
        if (abs(augmented(row, k)) > abs(augmented(pivot, k))) {
          pivot = row;
        }
      }

      if (is_zero(augmented(pivot, k), column_scale(a, k))) {
        throw std::invalid_argument("Matrix is singular!");
      }

      if (pivot != k) {
        for (std::size_t col = 0; col < n * 2; ++col) {
          std::swap(augmented(k, col), augmented(pivot, col));
        }
      }

      const T pivot_value = augmented(k, k);
      const std::size_t width = n * 2;
      bool scaled_with_simd = false;
      if constexpr (std::is_same<T, double>::value) {
        if (MATRIX_USE_SIMD && simd::enabled()) {
          simd::scale(augmented.data() + k * width + k,
                      augmented.data() + k * width + k, 1.0 / pivot_value,
                      width - k);
          scaled_with_simd = true;
        }
      }
      if (!scaled_with_simd) {
        MATRIX_SCALAR_FOR
        for (std::size_t col = k; col < width; ++col) {
          augmented(k, col) /= pivot_value;
        }
      }

      for (std::size_t row = 0; row < n; ++row) {
        if (row == k) {
          continue;
        }
        const T factor = augmented(row, k);
        bool eliminated_with_simd = false;
        if constexpr (std::is_same<T, double>::value) {
          if (MATRIX_USE_SIMD && simd::enabled()) {
            simd::axpy(augmented.data() + row * width + k,
                       augmented.data() + k * width + k, -factor, width - k);
            eliminated_with_simd = true;
          }
        }
        if (!eliminated_with_simd) {
          MATRIX_SCALAR_FOR
          for (std::size_t col = k; col < width; ++col) {
            augmented(row, col) -= factor * augmented(k, col);
          }
        }
      }
    }

    Matrix<T, Rows, Cols> result = [&] {
      if constexpr (Rows == 0 && Cols == 0) {
        return Matrix<T, Rows, Cols>(n, n);
      } else {
        return Matrix<T, Rows, Cols>{};
      }
    }();

    for (std::size_t row = 0; row < n; ++row) {
      for (std::size_t col = 0; col < n; ++col) {
        result(row, col) = augmented(row, n + col);
      }
    }

    return result;
  }
}

template <typename T>
const Matrix<T, 0, 0>& as_matrix(const DynamicSquareMatrix<T>& a) {
  return static_cast<const Matrix<T, 0, 0>&>(a);
}

template <typename T>
auto det(const DynamicSquareMatrix<T>& a) {
  return det(as_matrix(a));
}

template <typename T>
auto inverse(const DynamicSquareMatrix<T>& a) {
  return inverse(as_matrix(a));
}

template <typename T>
Matrix<T, 0, 0> transpose(const DynamicSquareMatrix<T>& a) {
  return transpose(as_matrix(a));
}

template <typename T>
Matrix<T, 0, 0> operator+(const DynamicSquareMatrix<T>& a,
                          const DynamicSquareMatrix<T>& b) {
  return as_matrix(a) + as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator+(const DynamicSquareMatrix<T>& a,
                          const Matrix<T, 0, 0>& b) {
  return as_matrix(a) + b;
}

template <typename T>
Matrix<T, 0, 0> operator+(const Matrix<T, 0, 0>& a,
                          const DynamicSquareMatrix<T>& b) {
  return a + as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator-(const DynamicSquareMatrix<T>& a,
                          const DynamicSquareMatrix<T>& b) {
  return as_matrix(a) - as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator-(const DynamicSquareMatrix<T>& a,
                          const Matrix<T, 0, 0>& b) {
  return as_matrix(a) - b;
}

template <typename T>
Matrix<T, 0, 0> operator-(const Matrix<T, 0, 0>& a,
                          const DynamicSquareMatrix<T>& b) {
  return a - as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator*(const DynamicSquareMatrix<T>& a,
                          const DynamicSquareMatrix<T>& b) {
  return as_matrix(a) * as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator*(const DynamicSquareMatrix<T>& a,
                          const Matrix<T, 0, 0>& b) {
  return as_matrix(a) * b;
}

template <typename T>
Matrix<T, 0, 0> operator*(const Matrix<T, 0, 0>& a,
                          const DynamicSquareMatrix<T>& b) {
  return a * as_matrix(b);
}

template <typename T>
Matrix<T, 0, 0> operator*(const DynamicSquareMatrix<T>& a, const T k) {
  return as_matrix(a) * k;
}

template <typename T>
Matrix<T, 0, 0> operator*(const T k, const DynamicSquareMatrix<T>& a) {
  return k * as_matrix(a);
}

// Transpose of a matrix.
template <typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Cols, Rows> transpose(const Matrix<T, Rows, Cols>& a) {
  Matrix<T, Cols, Rows> result = [&] {
    if constexpr (Rows == 0 && Cols == 0) {
      return Matrix<T, Cols, Rows>(a.cols(), a.rows());
    } else {
      return Matrix<T, Cols, Rows>{};
    }
  }();

  for (std::size_t row = 0; row < a.rows(); ++row) {
    for (std::size_t col = 0; col < a.cols(); ++col) {
      result(col, row) = a(row, col);
    }
  }
  return result;
}

// Identity matrix with a compile-time size.
template <typename T, std::size_t N>
Matrix<T, N, N> identity() {
  Matrix<T, N, N> result;
  for (std::size_t i = 0; i < N; ++i) {
    result(i, i) = one_element<T>();
  }
  return result;
}

// Identity matrix with a size set at run time.
template <typename T>
Matrix<T, 0, 0> identity(std::size_t n) {
  Matrix<T, 0, 0> result(n, n);
  for (std::size_t i = 0; i < n; ++i) {
    result(i, i) = one_element<T>();
  }
  return result;
}

// Givens reduction of one row, applied on the right. Rotates columns so that
// every entry after pivot_col in `row` becomes zero. An orthogonal rotation of
// the columns keeps M M^T unchanged, which square-root filters rely on.
// Works for static and dynamic matrices. Throws std::overflow_error when a
// rotated entry is not finite.
template <std::size_t Rows, std::size_t Cols>
void givens_reduce_row(Matrix<double, Rows, Cols>& m, std::size_t row,
                       std::size_t pivot_col) {
  for (std::size_t col = pivot_col + 1; col < m.cols(); ++col) {
    const double entry = m(row, col);
    if (entry == 0.0) {
      continue;
    }
    const double pivot = m(row, pivot_col);
    // Scale first, so subnormal pivots still give normalized coefficients.
    const double scale = std::max(std::abs(pivot), std::abs(entry));
    if (!std::isfinite(scale) || !std::isfinite(entry)) {
      throw std::overflow_error("Givens rotation overflowed");
    }
    const double x = pivot / scale;
    const double y = entry / scale;
    const double norm = std::hypot(x, y);
    const double cosine = x / norm;
    const double sine = y / norm;
    m(row, pivot_col) = scale * norm;
    m(row, col) = 0.0;
    for (std::size_t next = row + 1; next < m.rows(); ++next) {
      const double left = m(next, pivot_col);
      const double right = m(next, col);
      m(next, pivot_col) = cosine * left + sine * right;
      m(next, col) = cosine * right - sine * left;
    }
  }
}

// Horizontal block assembly: [left | right]. The row counts must match.
// For dynamic matrices the result is sized from the operands.
template <typename T, std::size_t Rows, std::size_t LeftCols,
          std::size_t RightCols>
Matrix<T, Rows, LeftCols + RightCols> hconcat(
    const Matrix<T, Rows, LeftCols>& left,
    const Matrix<T, Rows, RightCols>& right) {
  if (left.rows() != right.rows()) {
    throw std::invalid_argument("hconcat: row counts differ");
  }

  Matrix<T, Rows, LeftCols + RightCols> result = [&] {
    if constexpr (Rows == 0 && LeftCols + RightCols == 0) {
      return Matrix<T, Rows, LeftCols + RightCols>(
          left.rows(), left.cols() + right.cols());
    } else {
      return Matrix<T, Rows, LeftCols + RightCols>{};
    }
  }();

  for (std::size_t row = 0; row < left.rows(); ++row) {
    for (std::size_t col = 0; col < left.cols(); ++col) {
      result(row, col) = left(row, col);
    }
    for (std::size_t col = 0; col < right.cols(); ++col) {
      result(row, left.cols() + col) = right(row, col);
    }
  }
  return result;
}

// Vertical block assembly: [top; bottom]. The column counts must match.
// For dynamic matrices the result is sized from the operands.
template <typename T, std::size_t TopRows, std::size_t BottomRows,
          std::size_t Cols>
Matrix<T, TopRows + BottomRows, Cols> vconcat(
    const Matrix<T, TopRows, Cols>& top,
    const Matrix<T, BottomRows, Cols>& bottom) {
  if (top.cols() != bottom.cols()) {
    throw std::invalid_argument("vconcat: column counts differ");
  }

  Matrix<T, TopRows + BottomRows, Cols> result = [&] {
    if constexpr (TopRows + BottomRows == 0 && Cols == 0) {
      return Matrix<T, TopRows + BottomRows, Cols>(top.rows() + bottom.rows(),
                                                   top.cols());
    } else {
      return Matrix<T, TopRows + BottomRows, Cols>{};
    }
  }();

  for (std::size_t row = 0; row < top.rows(); ++row) {
    for (std::size_t col = 0; col < top.cols(); ++col) {
      result(row, col) = top(row, col);
    }
  }
  for (std::size_t row = 0; row < bottom.rows(); ++row) {
    for (std::size_t col = 0; col < bottom.cols(); ++col) {
      result(top.rows() + row, col) = bottom(row, col);
    }
  }
  return result;
}

#undef MATRIX_SCALAR_FOR

#endif  // SENSOR_FUSION_MATRIX_MATRIX_H_
