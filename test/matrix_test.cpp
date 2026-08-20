/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <gtest/gtest.h>
#include <math/matrix.hpp>

using namespace m5::math;

TEST(MatrixTest, DefaultIsZero)
{
    const Matrix<2, 3> m;
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(m(r, c), 0.0f);
        }
    }
}

TEST(MatrixTest, AtAndParenAccessMutate)
{
    Matrix<2, 2> m;
    m.at(0, 0) = 1.0f;
    m(0, 1)    = 2.0f;
    m.at(1, 0) = 3.0f;
    m(1, 1)    = 4.0f;

    EXPECT_FLOAT_EQ(m.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(m.at(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0f);
}

TEST(MatrixTest, AddAndSubtract)
{
    Matrix<2, 2> a;
    a(0, 0) = 1.0f;
    a(0, 1) = 2.0f;
    a(1, 0) = 3.0f;
    a(1, 1) = 4.0f;

    Matrix<2, 2> b;
    b(0, 0) = 5.0f;
    b(0, 1) = 6.0f;
    b(1, 0) = 7.0f;
    b(1, 1) = 8.0f;

    const Matrix<2, 2> sum = a + b;
    EXPECT_FLOAT_EQ(sum(0, 0), 6.0f);
    EXPECT_FLOAT_EQ(sum(0, 1), 8.0f);
    EXPECT_FLOAT_EQ(sum(1, 0), 10.0f);
    EXPECT_FLOAT_EQ(sum(1, 1), 12.0f);

    const Matrix<2, 2> diff = a - b;
    EXPECT_FLOAT_EQ(diff(0, 0), -4.0f);
    EXPECT_FLOAT_EQ(diff(0, 1), -4.0f);
    EXPECT_FLOAT_EQ(diff(1, 0), -4.0f);
    EXPECT_FLOAT_EQ(diff(1, 1), -4.0f);
}

TEST(MatrixTest, ScalarMultiply)
{
    Matrix<2, 2> a;
    a(0, 0) = 1.0f;
    a(0, 1) = -2.0f;
    a(1, 0) = 3.0f;
    a(1, 1) = -4.0f;

    const Matrix<2, 2> r = a * 2.0f;
    EXPECT_FLOAT_EQ(r(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(r(0, 1), -4.0f);
    EXPECT_FLOAT_EQ(r(1, 0), 6.0f);
    EXPECT_FLOAT_EQ(r(1, 1), -8.0f);
}

TEST(MatrixTest, ProductKnown2x3Times3x2)
{
    Matrix<2, 3> a;
    a(0, 0) = 1.0f;
    a(0, 1) = 2.0f;
    a(0, 2) = 3.0f;
    a(1, 0) = 4.0f;
    a(1, 1) = 5.0f;
    a(1, 2) = 6.0f;

    Matrix<3, 2> b;
    b(0, 0) = 7.0f;
    b(0, 1) = 8.0f;
    b(1, 0) = 9.0f;
    b(1, 1) = 10.0f;
    b(2, 0) = 11.0f;
    b(2, 1) = 12.0f;

    const Matrix<2, 2> product = a * b;
    EXPECT_FLOAT_EQ(product(0, 0), 58.0f);
    EXPECT_FLOAT_EQ(product(0, 1), 64.0f);
    EXPECT_FLOAT_EQ(product(1, 0), 139.0f);
    EXPECT_FLOAT_EQ(product(1, 1), 154.0f);
}

TEST(MatrixTest, Transposed)
{
    Matrix<2, 3> a;
    a(0, 0) = 1.0f;
    a(0, 1) = 2.0f;
    a(0, 2) = 3.0f;
    a(1, 0) = 4.0f;
    a(1, 1) = 5.0f;
    a(1, 2) = 6.0f;

    const Matrix<3, 2> t = a.transposed();
    EXPECT_FLOAT_EQ(t(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(t(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(t(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(t(1, 1), 5.0f);
    EXPECT_FLOAT_EQ(t(2, 0), 3.0f);
    EXPECT_FLOAT_EQ(t(2, 1), 6.0f);
}

TEST(MatrixTest, IdentitySquare)
{
    const Matrix<3, 3> id = Matrix<3, 3>::identity();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_FLOAT_EQ(id(r, c), (r == c) ? 1.0f : 0.0f);
        }
    }
}

TEST(MatrixTest, Invert3x3KnownDiagonalMatrixRecoversIdentity)
{
    Matrix<3, 3> m;
    m(0, 0) = 2.0f;
    m(1, 1) = 4.0f;
    m(2, 2) = 5.0f;

    Matrix<3, 3> inv;
    const bool ok = invert3x3(m, inv);
    EXPECT_TRUE(ok);

    EXPECT_NEAR(inv(0, 0), 0.5f, 1e-6f);
    EXPECT_NEAR(inv(1, 1), 0.25f, 1e-6f);
    EXPECT_NEAR(inv(2, 2), 0.2f, 1e-6f);
    EXPECT_NEAR(inv(0, 1), 0.0f, 1e-6f);
    EXPECT_NEAR(inv(0, 2), 0.0f, 1e-6f);
    EXPECT_NEAR(inv(1, 2), 0.0f, 1e-6f);

    const Matrix<3, 3> product = m * inv;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(product(r, c), (r == c) ? 1.0f : 0.0f, 1e-5f);
        }
    }
}

TEST(MatrixTest, Invert3x3NonDiagonalMatrixRecoversIdentity)
{
    // Arbitrary invertible (non-symmetric, non-diagonal) matrix.
    Matrix<3, 3> m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(0, 2) = 3.0f;
    m(1, 0) = 0.0f;
    m(1, 1) = 1.0f;
    m(1, 2) = 4.0f;
    m(2, 0) = 5.0f;
    m(2, 1) = 6.0f;
    m(2, 2) = 0.0f;

    Matrix<3, 3> inv;
    const bool ok = invert3x3(m, inv);
    EXPECT_TRUE(ok);

    const Matrix<3, 3> product = m * inv;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(product(r, c), (r == c) ? 1.0f : 0.0f, 1e-4f);
        }
    }
}

TEST(MatrixTest, Invert3x3SingularReturnsFalse)
{
    // Row 1 = 2 * Row 0: singular (determinant is zero).
    Matrix<3, 3> m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(0, 2) = 3.0f;
    m(1, 0) = 2.0f;
    m(1, 1) = 4.0f;
    m(1, 2) = 6.0f;
    m(2, 0) = 7.0f;
    m(2, 1) = 8.0f;
    m(2, 2) = 9.0f;

    Matrix<3, 3> inv;
    const bool ok = invert3x3(m, inv);
    EXPECT_FALSE(ok);
}

TEST(MatrixTest, CholeskySolveKnownSPDSystem)
{
    // Classic textbook SPD matrix (A = L * L^T with L = [[2,0,0],[6,1,0],[-8,5,3]]).
    Matrix<3, 3> a;
    a(0, 0) = 4.0f;
    a(0, 1) = 12.0f;
    a(0, 2) = -16.0f;
    a(1, 0) = 12.0f;
    a(1, 1) = 37.0f;
    a(1, 2) = -43.0f;
    a(2, 0) = -16.0f;
    a(2, 1) = -43.0f;
    a(2, 2) = 98.0f;

    // b = A * [1, 2, 3]^T
    Matrix<3, 1> b;
    b(0, 0) = -20.0f;
    b(1, 0) = -43.0f;
    b(2, 0) = 192.0f;

    Matrix<3, 1> x;
    const bool ok = cholesky_solve(a, b, x);
    EXPECT_TRUE(ok);

    EXPECT_NEAR(x(0, 0), 1.0f, 1e-3f);
    EXPECT_NEAR(x(1, 0), 2.0f, 1e-3f);
    EXPECT_NEAR(x(2, 0), 3.0f, 1e-3f);
}

TEST(MatrixTest, CholeskySolveNonPositiveDefiniteReturnsFalse)
{
    Matrix<3, 3> a;
    a(0, 0) = -1.0f;
    a(1, 1) = 1.0f;
    a(2, 2) = 1.0f;

    Matrix<3, 1> b;
    b(0, 0) = 1.0f;
    b(1, 0) = 1.0f;
    b(2, 0) = 1.0f;

    Matrix<3, 1> x;
    const bool ok = cholesky_solve(a, b, x);
    EXPECT_FALSE(ok);
}
