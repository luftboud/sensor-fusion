#include "matrix/matrix.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

int g_failures = 0;

void check(bool ok, const char* text, int line) {
    if (!ok) {
        std::cerr << "line " << line << ": " << text << '\n';
        ++g_failures;
    }
}

#define CHECK(condition) check(static_cast<bool>(condition), #condition, __LINE__)

bool close(double actual, double expected) {
    return std::abs(actual - expected) < 1e-9;
}

template <typename Function>
void expect_invalid_argument(Function&& function) {
    bool threw = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

void test_elements() {
    CHECK(zero_element<int>() == 0);
    CHECK(zero_element<double>() == 0.0);
    CHECK(one_element<int>() == 1);
    CHECK(one_element<double>() == 1.0);

    static_assert(Matrix<double, 2, 3>::rows() == 2);
    static_assert(Matrix<double, 2, 3>::cols() == 3);
    static_assert(Vector<double, 4>::rows() == 4);
    static_assert(Vector<double, 4>::cols() == 1);
    static_assert(SquareMatrix<int, 3>::rows() == 3);
    static_assert(SquareMatrix<int, 3>::cols() == 3);
}

void test_static_access() {
    Matrix<double, 2, 3> matrix;
    CHECK(matrix.rows() == 2);
    CHECK(matrix.cols() == 3);
    CHECK(matrix(0, 0) == 0.0);
    CHECK(matrix(1, 2) == 0.0);

    matrix(1, 2) = 4.0;
    CHECK(matrix.data()[5] == 4.0);

    const Matrix<double, 2, 3>& view = matrix;
    CHECK(view.rows() == 2);
    CHECK(view.cols() == 3);
    CHECK(view(1, 2) == 4.0);
    CHECK(view.data()[5] == 4.0);
}

void test_static_arithmetic() {
    Matrix<double, 2, 2> left;
    Matrix<double, 2, 2> right;
    left(0, 0) = 1;
    left(0, 1) = 2;
    left(1, 0) = 3;
    left(1, 1) = 4;
    right(0, 0) = 5;
    right(0, 1) = 6;
    right(1, 0) = 7;
    right(1, 1) = 8;

    const Matrix<double, 2, 2> sum = left + right;
    CHECK(close(sum(0, 0), 6));
    CHECK(close(sum(0, 1), 8));
    CHECK(close(sum(1, 0), 10));
    CHECK(close(sum(1, 1), 12));

    const Matrix<double, 2, 2> difference = left - right;
    CHECK(close(difference(0, 0), -4));
    CHECK(close(difference(0, 1), -4));
    CHECK(close(difference(1, 0), -4));
    CHECK(close(difference(1, 1), -4));

    const Matrix<double, 2, 2> scaled = left * 2.0;
    const Matrix<double, 2, 2> scaled_from_left = 3.0 * left;
    CHECK(close(scaled(0, 0), 2));
    CHECK(close(scaled(1, 1), 8));
    CHECK(close(scaled_from_left(0, 1), 6));
    CHECK(close(scaled_from_left(1, 0), 9));

    Matrix<double, 2, 3> wide;
    Matrix<double, 3, 2> tall;
    wide(0, 0) = 1;
    wide(0, 1) = 2;
    wide(0, 2) = 3;
    wide(1, 0) = 4;
    wide(1, 1) = 5;
    wide(1, 2) = 6;
    tall(0, 0) = 7;
    tall(0, 1) = 8;
    tall(1, 0) = 9;
    tall(1, 1) = 10;
    tall(2, 0) = 11;
    tall(2, 1) = 12;

    const Matrix<double, 2, 2> product = wide * tall;
    CHECK(product.rows() == 2);
    CHECK(product.cols() == 2);
    CHECK(close(product(0, 0), 58));
    CHECK(close(product(0, 1), 64));
    CHECK(close(product(1, 0), 139));
    CHECK(close(product(1, 1), 154));
}

void test_static_det_and_inverse() {
    SquareMatrix<double, 2> matrix;
    matrix(0, 0) = 1;
    matrix(0, 1) = 2;
    matrix(1, 0) = 3;
    matrix(1, 1) = 4;
    CHECK(close(det(matrix), -2));

    const SquareMatrix<double, 2> inverted = inverse(matrix);
    const SquareMatrix<double, 2> identity = matrix * inverted;
    CHECK(close(identity(0, 0), 1));
    CHECK(close(identity(0, 1), 0));
    CHECK(close(identity(1, 0), 0));
    CHECK(close(identity(1, 1), 1));

    SquareMatrix<double, 2> swapped;
    swapped(0, 0) = 0;
    swapped(0, 1) = 1;
    swapped(1, 0) = 1;
    swapped(1, 1) = 0;
    CHECK(close(det(swapped), -1));

    SquareMatrix<double, 3> larger;
    larger(0, 0) = 1;
    larger(0, 1) = 2;
    larger(0, 2) = 3;
    larger(1, 0) = 0;
    larger(1, 1) = 1;
    larger(1, 2) = 4;
    larger(2, 0) = 5;
    larger(2, 1) = 6;
    larger(2, 2) = 0;
    CHECK(close(det(larger), 1));
    const SquareMatrix<double, 3> larger_identity = larger * inverse(larger);
    CHECK(close(larger_identity(0, 0), 1));
    CHECK(close(larger_identity(1, 1), 1));
    CHECK(close(larger_identity(2, 2), 1));
    CHECK(close(larger_identity(0, 1), 0));
    CHECK(close(larger_identity(2, 0), 0));

    SquareMatrix<double, 2> singular;
    singular(0, 0) = 1;
    singular(0, 1) = 2;
    singular(1, 0) = 2;
    singular(1, 1) = 4;
    CHECK(close(det(singular), 0));
    expect_invalid_argument([&] { (void)inverse(singular); });

    Matrix<double, 2, 3> rectangular;
    expect_invalid_argument([&] { (void)det(rectangular); });
    expect_invalid_argument([&] { (void)inverse(rectangular); });

    SquareMatrix<int, 2> integer_identity;
    integer_identity(0, 0) = 1;
    integer_identity(1, 1) = 1;
    CHECK(det(integer_identity) == 1);
    const SquareMatrix<int, 2> integer_inverse = inverse(integer_identity);
    CHECK(integer_inverse(0, 0) == 1);
    CHECK(integer_inverse(0, 1) == 0);
    CHECK(integer_inverse(1, 0) == 0);
    CHECK(integer_inverse(1, 1) == 1);
}

void test_dynamic_access() {
    const Matrix<double> matrix(2, 3);
    CHECK(matrix.rows() == 2);
    CHECK(matrix.cols() == 3);
    CHECK(matrix(0, 0) == 0.0);
    CHECK(matrix(1, 2) == 0.0);
    CHECK(matrix.data()[0] == 0.0);

    Matrix<double> mutable_matrix(2, 3);
    mutable_matrix(1, 2) = 4.0;
    CHECK(mutable_matrix.data()[5] == 4.0);

    const Matrix<double>& view = mutable_matrix;
    CHECK(view(1, 2) == 4.0);
    CHECK(view.data()[5] == 4.0);
}

void test_dynamic_arithmetic() {
    Matrix<double> left(2, 2);
    Matrix<double> right(2, 2);
    left(0, 0) = 1;
    left(0, 1) = 2;
    left(1, 0) = 3;
    left(1, 1) = 4;
    right(0, 0) = 5;
    right(0, 1) = 6;
    right(1, 0) = 7;
    right(1, 1) = 8;

    const Matrix<double> sum = left + right;
    CHECK(sum.rows() == 2);
    CHECK(sum.cols() == 2);
    CHECK(close(sum(0, 0), 6));
    CHECK(close(sum(1, 1), 12));

    const Matrix<double> difference = left - right;
    CHECK(close(difference(0, 0), -4));
    CHECK(close(difference(1, 1), -4));

    const Matrix<double> scaled = left * 2.0;
    const Matrix<double> scaled_from_left = 3.0 * left;
    CHECK(close(scaled(1, 0), 6));
    CHECK(close(scaled_from_left(0, 1), 6));

    Matrix<double> wide(2, 3);
    Matrix<double> tall(3, 2);
    wide(0, 0) = 1;
    wide(0, 1) = 2;
    wide(0, 2) = 3;
    wide(1, 0) = 4;
    wide(1, 1) = 5;
    wide(1, 2) = 6;
    tall(0, 0) = 7;
    tall(0, 1) = 8;
    tall(1, 0) = 9;
    tall(1, 1) = 10;
    tall(2, 0) = 11;
    tall(2, 1) = 12;

    const Matrix<double> product = wide * tall;
    CHECK(product.rows() == 2);
    CHECK(product.cols() == 2);
    CHECK(close(product(0, 0), 58));
    CHECK(close(product(0, 1), 64));
    CHECK(close(product(1, 0), 139));
    CHECK(close(product(1, 1), 154));

    Matrix<double> other_shape(2, 3);
    expect_invalid_argument([&] { (void)(left + other_shape); });
    expect_invalid_argument([&] { (void)(left - other_shape); });
    Matrix<double> bad_product(4, 2);
    expect_invalid_argument([&] { (void)(left * bad_product); });
}

void test_dynamic_det_and_inverse() {
    Matrix<double> matrix(2, 2);
    matrix(0, 0) = 1;
    matrix(0, 1) = 2;
    matrix(1, 0) = 3;
    matrix(1, 1) = 4;
    CHECK(close(det(matrix), -2));

    const Matrix<double> identity = matrix * inverse(matrix);
    CHECK(identity.rows() == 2);
    CHECK(close(identity(0, 0), 1));
    CHECK(close(identity(0, 1), 0));
    CHECK(close(identity(1, 0), 0));
    CHECK(close(identity(1, 1), 1));

    Matrix<double> singular(2, 2);
    singular(0, 0) = 1;
    singular(0, 1) = 2;
    singular(1, 0) = 2;
    singular(1, 1) = 4;
    CHECK(close(det(singular), 0));
    expect_invalid_argument([&] { (void)inverse(singular); });

    Matrix<double> rectangular(2, 3);
    expect_invalid_argument([&] { (void)det(rectangular); });
    expect_invalid_argument([&] { (void)inverse(rectangular); });
}

void test_float_arithmetic() {
    Matrix<float, 2, 2> left;
    Matrix<float, 2, 2> right;
    left(0, 0) = 1.f;
    left(1, 1) = 1.f;
    right(0, 0) = 2.f;
    right(0, 1) = 1.f;
    right(1, 0) = 3.f;
    right(1, 1) = 4.f;

    const Matrix<float, 2, 2> sum = left + right;
    CHECK(sum(0, 0) == 3.f);
    CHECK(sum(1, 1) == 5.f);

    const Matrix<float, 2, 2> difference = right - left;
    CHECK(difference(0, 0) == 1.f);
    CHECK(difference(1, 1) == 3.f);

    const Matrix<float, 2, 2> scaled = 2.f * right;
    CHECK(scaled(1, 0) == 6.f);
    CHECK(close(det(right), 5.0));
}

}

int main() {
    test_elements();
    test_static_access();
    test_static_arithmetic();
    test_static_det_and_inverse();
    test_dynamic_access();
    test_dynamic_arithmetic();
    test_dynamic_det_and_inverse();
    test_float_arithmetic();

    if (g_failures != 0) {
        std::cerr << g_failures << " checks failed\n";
        return 1;
    }

    std::cout << "All tests passed" << '\n';

    return 0;
}
