/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file vector3.hpp
  @brief Minimal 3D vector arithmetic (framework independent)
*/
#ifndef M5_UNIT_IMU_MATH_VECTOR3_HPP
#define M5_UNIT_IMU_MATH_VECTOR3_HPP

#include <cmath>

namespace m5 {
namespace math {

namespace {
//! @brief Vectors shorter than this are treated as zero length (normalize safety)
constexpr float VECTOR3_ZERO_EPSILON{1e-9f};
}  // namespace

/*!
  @struct Vector3
  @brief Plain 3D vector (float)
 */
struct Vector3 {
    float x{};  //!< @brief X component
    float y{};  //!< @brief Y component
    float z{};  //!< @brief Z component

    Vector3() = default;
    //! @brief Construct from components
    constexpr Vector3(const float x_, const float y_, const float z_) : x(x_), y(y_), z(z_)
    {
    }

    //! @brief Component-wise addition
    inline Vector3 operator+(const Vector3& rhs) const
    {
        return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
    }
    //! @brief Component-wise subtraction
    inline Vector3 operator-(const Vector3& rhs) const
    {
        return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
    }
    //! @brief Scalar multiplication
    inline Vector3 operator*(const float scalar) const
    {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    //! @brief Dot product
    inline float dot(const Vector3& rhs) const
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    //! @brief Cross product (right-handed)
    inline Vector3 cross(const Vector3& rhs) const
    {
        return Vector3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
    }
    //! @brief Euclidean norm (magnitude)
    inline float norm() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    /*!
      @brief Normalize this vector in place
      @return false and leaves the vector unchanged if it is (near) zero length
     */
    bool normalize()
    {
        const float n = norm();
        if (n < VECTOR3_ZERO_EPSILON) {
            return false;
        }
        const float inv = 1.0f / n;
        x *= inv;
        y *= inv;
        z *= inv;
        return true;
    }
    //! @brief Return a normalized copy (zero vector maps to zero vector)
    inline Vector3 normalized() const
    {
        Vector3 v(*this);
        v.normalize();
        return v;
    }
};

}  // namespace math
}  // namespace m5
#endif
