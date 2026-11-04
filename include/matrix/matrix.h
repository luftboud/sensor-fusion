#ifndef SENSOR_FUSION_MATRIX_H
#define SENSOR_FUSION_MATRIX_H


#include "simd.h"

#include <array>
#include <utility>
#include <vector>
#include <cstddef>
#include <stdexcept>
#include <cmath>
#include <type_traits>

#if defined(__clang__)
#define MATRIX_SCALAR_FOR _Pragma("clang loop vectorize(disable) interleave(disable)")
#elif defined(__GNUC__) && (__GNUC__ >= 14)
#define MATRIX_SCALAR_FOR _Pragma("GCC novector")
#else
#define MATRIX_SCALAR_FOR
#endif

// static matrix
template<typename T, std::size_t Rows = 0, std::size_t Cols = 0>
class Matrix {
public:
    Matrix() {
        for (T& element : data_)
            element = T();
    }

    static constexpr std::size_t rows() {return Rows;}
    static constexpr std::size_t cols() {return Cols;}

    T& operator()(std::size_t row, std::size_t col){
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

// dynamic matrix
template<typename T>
class Matrix<T, 0, 0> {
public:
    Matrix(std::size_t rows, std::size_t cols)
        : rows_(rows), cols_(cols), data_(rows * cols) {}

    std::size_t rows() const {return rows_;}
    std::size_t cols() const {return cols_;}

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

// vector
template<typename T, std::size_t N>
using Vector = Matrix<T, N, 1>;

// square matrix
template<typename T, std::size_t N>
using SquareMatrix = Matrix<T, N, N>;

template<typename T>
T zero_element() {
    return T();
}

template<typename T>
T one_element() {
    if constexpr (std::is_floating_point<T>::value || std::is_integral<T>::value)
        return static_cast<T>(1);
    else // for ratio
        return T(1, 1);
}

// adding of two matrices
template<typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols>& a,
                                const Matrix<T, Rows, Cols>& b){
    if (a.cols() != b.cols() || a.rows() != b.rows()) {
        throw std::invalid_argument("Matrix sizes differ!");
    }

    Matrix<T, Rows, Cols> result = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols>(a.rows(), a.cols());
        else
            return Matrix<T, Rows, Cols>{};
    }();

    const std::size_t count = a.rows() * a.cols();
    if constexpr (std::is_same<T, double>::value) {
        if (simd::enabled()) {
            simd::add(result.data(), a.data(), b.data(), count);
            return result;
        }
    }
    MATRIX_SCALAR_FOR
    for (std::size_t r = 0; r < a.rows(); ++r) {
        MATRIX_SCALAR_FOR
        for (std::size_t c = 0; c < a.cols(); ++c)
            result(r, c) = a(r, c) + b(r, c);
    }
    return result;
}

// substructing matrix b from matrix a
template<typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator-(const Matrix<T, Rows, Cols>& a,
                                const Matrix<T, Rows, Cols>& b){
    if (a.cols() != b.cols() || a.rows() != b.rows()) {
        throw std::invalid_argument("Matrix sizes differ!");
    }

    Matrix<T, Rows, Cols> result = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols>(a.rows(), a.cols());
        else
            return Matrix<T, Rows, Cols>{};
    }();

    const std::size_t count = a.rows() * a.cols();
    if constexpr (std::is_same<T, double>::value) {
        if (simd::enabled()) {
            simd::sub(result.data(), a.data(), b.data(), count);
            return result;
        }
    }
    MATRIX_SCALAR_FOR
    for (std::size_t r = 0; r < a.rows(); ++r) {
        MATRIX_SCALAR_FOR
        for (std::size_t c = 0; c < a.cols(); ++c)
            result(r, c) = a(r, c) - b(r, c);
    }
    return result;
}

// multiplication by a scalar
template<typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const Matrix<T, Rows, Cols>& a, const T k) {
    Matrix<T, Rows, Cols> result = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols>(a.rows(), a.cols());
        else
            return Matrix<T, Rows, Cols>{};
    }();

    const std::size_t count = a.rows() * a.cols();
    if constexpr (std::is_same<T, double>::value) {
        if (simd::enabled()) {
            simd::scale(result.data(), a.data(), k, count);
            return result;
        }
    }
    MATRIX_SCALAR_FOR
    for (std::size_t r = 0; r < a.rows(); ++r) {
        MATRIX_SCALAR_FOR
        for (std::size_t c = 0; c < a.cols(); ++c)
            result(r, c) = k * a(r, c);
    }
    return result;
}

template<typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const T k, const Matrix<T, Rows, Cols>& a) {
    return a*k;
}

// multiplication of two matrices
template<typename T, std::size_t Rows, std::size_t Inner, std::size_t Cols>
Matrix<T, Rows, Cols> operator*(const Matrix<T, Rows, Inner>& a, const Matrix<T, Inner, Cols>& b) {
    if (a.cols() != b.rows()) {
        throw std::invalid_argument("Incorrect matrix sizes!");
    }

    Matrix<T, Rows, Cols> result = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols>(a.rows(), b.cols());
        else
            return Matrix<T, Rows, Cols>{};
    }();

    if constexpr (std::is_same<T, double>::value) {
        if (simd::enabled()) {
            const std::size_t cols = b.cols();
            for (std::size_t row = 0; row < a.rows(); ++row) {
                double* destination = result.data() + row * cols;
                for (std::size_t k = 0; k < a.cols(); ++k)
                    simd::axpy(destination, b.data() + k * cols, a(row, k), cols);
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
            for (std::size_t k = 0; k < a.cols(); ++k)
                sum += a(fixed_row, k) * b(k, fixed_col);
            result(fixed_row, fixed_col) = sum;
        }
    }
    return result;
}
// determinant
template<typename T, std::size_t Rows, std::size_t Cols>
T det(const Matrix<T, Rows, Cols>& a) {
    if (a.cols() != a.rows()) {
        throw std::invalid_argument("Incorrect matrix size: matrix has to be square!");
    }

    Matrix<T, Rows, Cols> m = a;
    const std::size_t n = m.rows();
    T result = one_element<T>();

    using std::abs;

    for (std::size_t k = 0; k < n; ++k) {
        std::size_t pivot = k;
        for (std::size_t row = k+1; row < n; ++row){
            if (abs(m(row, k)) > abs(m(pivot, k)))
            pivot = row;
        }

        if (m(pivot, k) == zero_element<T>())
            return zero_element<T>();

        if (pivot != k) {
            for (std::size_t col = 0; col < n; ++col) {
                std::swap(m(k, col), m(pivot, col));
            }
            result = -result;
        }

        for (std::size_t row = k+1; row < n; ++row) {
            const T factor = m(row, k) / m(k, k);
            bool used_simd = false;
            if constexpr (std::is_same<T, double>::value) {
                if (simd::enabled()) {
                    simd::axpy(m.data() + row * n + k, m.data() + k * n + k, -factor, n - k);
                    used_simd = true;
                }
            }
            if (!used_simd) {
                MATRIX_SCALAR_FOR
                for (std::size_t col = k; col < n; ++col)
                    m(row, col) -= factor * m(k, col);
            }
        }
    }

    for (std::size_t i = 0; i < n; ++i)
        result *= m(i, i);

    return result;
}
// matrix inverse
template<typename T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols> inverse(const Matrix<T, Rows, Cols>& a) {
    if (a.rows() != a.cols())
        throw std::invalid_argument("Incorrect matrix size: matrix has to be square!");

    const std::size_t n = a.rows();

    Matrix<T, Rows, Cols*2> augmented = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols*2>(n, n*2);
        else
            return Matrix<T, Rows, Cols*2>{};
    }();

    for (std::size_t row = 0; row < n; ++row){
        for (std::size_t col = 0; col < n; ++col){
            augmented(row, col) = a(row, col);
        }
        augmented(row, n+row) = one_element<T>();
    }

    using std::abs;

    for (std::size_t k = 0; k < n; ++k){
        std::size_t pivot = k;
        for (std::size_t row = k+1; row < n; ++row) {
            if (abs(augmented(row, k)) > abs(augmented(pivot, k)))
                pivot = row;
        }

        if (augmented(pivot, k) == zero_element<T>())
            throw std::invalid_argument("Matrix is singular!");

        if (pivot != k) {
            for (std::size_t col = 0; col < n*2; ++col) {
                std::swap(augmented(k, col), augmented(pivot, col));
            }
        }

        const T pivot_value = augmented(k, k);
        const std::size_t width = n * 2;
        bool scaled_with_simd = false;
        if constexpr (std::is_same<T, double>::value) {
            if (simd::enabled()) {
                simd::scale(augmented.data() + k * width + k,
                            augmented.data() + k * width + k,
                            1.0 / pivot_value,
                            width - k);
                scaled_with_simd = true;
            }
        }
        if (!scaled_with_simd) {
            MATRIX_SCALAR_FOR
            for (std::size_t col = k; col < width; ++col)
                augmented(k, col) /= pivot_value;
        }

        for (std::size_t row = 0; row < n; ++row) {
            if (row == k)
                continue;
            const T factor = augmented(row, k);
            bool eliminated_with_simd = false;
            if constexpr (std::is_same<T, double>::value) {
                if (simd::enabled()) {
                    simd::axpy(augmented.data() + row * width + k,
                               augmented.data() + k * width + k,
                               -factor,
                               width - k);
                    eliminated_with_simd = true;
                }
            }
            if (!eliminated_with_simd) {
                MATRIX_SCALAR_FOR
                for (std::size_t col = k; col < width; ++col)
                    augmented(row, col) -= factor * augmented(k, col);
            }
        }
    }

    Matrix<T, Rows, Cols> result = [&] {
        if constexpr (Rows == 0 && Cols == 0)
            return Matrix<T, Rows, Cols>(n, n);
        else
            return Matrix<T, Rows, Cols>{};
    }();

    for (std::size_t row = 0; row < n; ++row){
        for (std::size_t col = 0; col < n; ++col) {
            result(row, col) = augmented(row, n + col);
        }
    }

    return result;
}
#undef MATRIX_SCALAR_FOR

#endif //SENSOR_FUSION_MATRIX_H
