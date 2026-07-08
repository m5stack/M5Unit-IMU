/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file matrix.hpp
  @brief Minimal fixed-size matrix arithmetic (framework independent)
  @details Header-only, heap-free (std::array backed) fixed-size matrix type,
  providing only what src/imu/filter/ekf.hpp needs: element access, addition,
  subtraction, scalar and matrix multiplication, transpose, and identity for
  square matrices. General NxN inversion is intentionally not provided;
  instead two narrow numerical primitives are exposed as free functions:
  - invert3x3(): closed-form 3x3 inverse, for a single 3-axis observation
    update (e.g. accel, then mag as a separate sequential step)
  - cholesky_solve(): solves A*x = b for a symmetric positive-definite A of
    any fixed size via Cholesky decomposition, for callers that prefer a
    single combined observation update (e.g. a 6x6 accel+mag innovation
    covariance) instead of two sequential 3x3 updates
*/
#ifndef M5_UNIT_IMU_MATH_MATRIX_HPP
#define M5_UNIT_IMU_MATH_MATRIX_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace m5 {
namespace math {

//! @brief Magnitudes below this are treated as singular/non-positive-definite (matrix safety)
constexpr float MATRIX_ZERO_EPSILON{1e-9f};

/*!
  @class Matrix
  @brief Fixed-size (R rows x C columns) float matrix, std::array backed (no heap allocation)
 */
template <std::size_t R, std::size_t C>
class Matrix {
public:
    Matrix() = default;

    //! @brief Number of rows
    static constexpr std::size_t rows()
    {
        return R;
    }
    //! @brief Number of columns
    static constexpr std::size_t cols()
    {
        return C;
    }

    //! @brief Element access (mutable)
    inline float& at(const std::size_t row, const std::size_t col)
    {
        return _data[row][col];
    }
    //! @brief Element access (read-only)
    inline float at(const std::size_t row, const std::size_t col) const
    {
        return _data[row][col];
    }
    //! @brief Element access via operator() (mutable)
    inline float& operator()(const std::size_t row, const std::size_t col)
    {
        return at(row, col);
    }
    //! @brief Element access via operator() (read-only)
    inline float operator()(const std::size_t row, const std::size_t col) const
    {
        return at(row, col);
    }

    //! @brief Component-wise addition
    inline Matrix<R, C> operator+(const Matrix<R, C>& rhs) const
    {
        Matrix<R, C> result;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                result._data[r][c] = _data[r][c] + rhs._data[r][c];
            }
        }
        return result;
    }
    //! @brief Component-wise subtraction
    inline Matrix<R, C> operator-(const Matrix<R, C>& rhs) const
    {
        Matrix<R, C> result;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                result._data[r][c] = _data[r][c] - rhs._data[r][c];
            }
        }
        return result;
    }
    //! @brief Scalar multiplication
    inline Matrix<R, C> operator*(const float scalar) const
    {
        Matrix<R, C> result;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                result._data[r][c] = _data[r][c] * scalar;
            }
        }
        return result;
    }

    //! @brief Matrix product (this [R x C] times rhs [C x K] -> [R x K])
    template <std::size_t K>
    inline Matrix<R, K> operator*(const Matrix<C, K>& rhs) const
    {
        Matrix<R, K> result;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t k = 0; k < K; ++k) {
                float sum{};
                for (std::size_t c = 0; c < C; ++c) {
                    sum += _data[r][c] * rhs(c, k);
                }
                result(r, k) = sum;
            }
        }
        return result;
    }

    //! @brief Transpose (C x R)
    inline Matrix<C, R> transposed() const
    {
        Matrix<C, R> result;
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                result(c, r) = _data[r][c];
            }
        }
        return result;
    }

    //! @brief Identity matrix (square matrices only)
    static inline Matrix<R, C> identity()
    {
        static_assert(R == C, "Matrix::identity() requires a square matrix");
        Matrix<R, C> result;
        for (std::size_t i = 0; i < R; ++i) {
            result._data[i][i] = 1.0f;
        }
        return result;
    }

private:
    std::array<std::array<float, C>, R> _data{};
};

/*!
  @brief Closed-form inverse of a 3x3 matrix
  @param m Matrix to invert
  @param[out] out Inverse of m; only written on success
  @return False if m is (near) singular (determinant magnitude below MATRIX_ZERO_EPSILON),
  leaving out unwritten
 */
inline bool invert3x3(const Matrix<3, 3>& m, Matrix<3, 3>& out)
{
    const float a = m(0, 0);
    const float b = m(0, 1);
    const float c = m(0, 2);
    const float d = m(1, 0);
    const float e = m(1, 1);
    const float f = m(1, 2);
    const float g = m(2, 0);
    const float h = m(2, 1);
    const float i = m(2, 2);

    const float cof00 = e * i - f * h;
    const float cof01 = -(d * i - f * g);
    const float cof02 = d * h - e * g;
    const float cof10 = -(b * i - c * h);
    const float cof11 = a * i - c * g;
    const float cof12 = -(a * h - b * g);
    const float cof20 = b * f - c * e;
    const float cof21 = -(a * f - c * d);
    const float cof22 = a * e - b * d;

    const float det = a * cof00 + b * cof01 + c * cof02;
    if (std::fabs(det) < MATRIX_ZERO_EPSILON) {
        return false;
    }
    const float inv_det = 1.0f / det;

    // Adjugate (transpose of the cofactor matrix) scaled by 1/det.
    out(0, 0) = cof00 * inv_det;
    out(0, 1) = cof10 * inv_det;
    out(0, 2) = cof20 * inv_det;
    out(1, 0) = cof01 * inv_det;
    out(1, 1) = cof11 * inv_det;
    out(1, 2) = cof21 * inv_det;
    out(2, 0) = cof02 * inv_det;
    out(2, 1) = cof12 * inv_det;
    out(2, 2) = cof22 * inv_det;
    return true;
}

/*!
  @brief Solve A*x = b for a symmetric positive-definite A via Cholesky decomposition
  @param a Symmetric positive-definite matrix (only the lower triangle, row >= col, is read)
  @param b Right-hand side column vector
  @param[out] x Solution; only written on success
  @return False if a is not (numerically) positive-definite, leaving x unwritten
 */
template <std::size_t N>
inline bool cholesky_solve(const Matrix<N, N>& a, const Matrix<N, 1>& b, Matrix<N, 1>& x)
{
    std::array<std::array<float, N>, N> l{};

    for (std::size_t row = 0; row < N; ++row) {
        for (std::size_t col = 0; col <= row; ++col) {
            float sum = a(row, col);
            for (std::size_t k = 0; k < col; ++k) {
                sum -= l[row][k] * l[col][k];
            }
            if (row == col) {
                if (sum < MATRIX_ZERO_EPSILON) {
                    return false;
                }
                l[row][col] = std::sqrt(sum);
            } else {
                l[row][col] = sum / l[col][col];
            }
        }
    }

    // Forward substitution: L * y = b
    std::array<float, N> y{};
    for (std::size_t row = 0; row < N; ++row) {
        float sum = b(row, 0);
        for (std::size_t k = 0; k < row; ++k) {
            sum -= l[row][k] * y[k];
        }
        y[row] = sum / l[row][row];
    }

    // Back substitution: L^T * x = y
    for (std::size_t ridx = N; ridx-- > 0;) {
        float sum = y[ridx];
        for (std::size_t k = ridx + 1; k < N; ++k) {
            sum -= l[k][ridx] * x(k, 0);
        }
        x(ridx, 0) = sum / l[ridx][ridx];
    }
    return true;
}

}  // namespace math
}  // namespace m5
#endif
