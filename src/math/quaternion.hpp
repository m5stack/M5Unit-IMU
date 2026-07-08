/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file quaternion.hpp
  @brief Minimal quaternion arithmetic for attitude representation (framework independent)
*/
#ifndef M5_UNIT_IMU_MATH_QUATERNION_HPP
#define M5_UNIT_IMU_MATH_QUATERNION_HPP

#include <cmath>

#include "vector3.hpp"

namespace m5 {
namespace math {

namespace {
//! @brief Quaternions with a norm below this are treated as zero (normalize safety)
constexpr float QUATERNION_ZERO_EPSILON{1e-9f};
//! @brief PI (float)
constexpr float QUATERNION_PI{3.14159265358979323846f};
}  // namespace

/*!
  @struct Quaternion
  @brief Plain Hamilton quaternion (float) used to represent attitude
  @details Euler angle conversions use the aerospace ZYX (yaw-pitch-roll) convention, in degrees.
  Rotations are right-handed and active (rotate a vector, not the frame).
 */
struct Quaternion {
    float w{1.0f};  //!< @brief Scalar part
    float x{};      //!< @brief Vector part X
    float y{};      //!< @brief Vector part Y
    float z{};      //!< @brief Vector part Z

    //! @brief Default-construct as identity (no rotation)
    Quaternion() = default;
    //! @brief Construct from components
    constexpr Quaternion(const float w_, const float x_, const float y_, const float z_) : w(w_), x(x_), y(y_), z(z_)
    {
    }

    //! @brief Identity quaternion (no rotation)
    static inline Quaternion identity()
    {
        return Quaternion();
    }

    //! @brief Hamilton product (this rotation applied after rhs)
    inline Quaternion operator*(const Quaternion& rhs) const
    {
        return Quaternion(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z, w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                          w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x, w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
    }

    //! @brief Conjugate (equals the inverse for a unit quaternion)
    inline Quaternion conjugated() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    //! @brief Squared norm
    inline float normSq() const
    {
        return w * w + x * x + y * y + z * z;
    }
    //! @brief Norm (magnitude)
    inline float norm() const
    {
        return std::sqrt(normSq());
    }
    /*!
      @brief Normalize this quaternion in place
      @return false and leaves the quaternion unchanged if it is (near) zero
     */
    bool normalize()
    {
        const float n = norm();
        if (n < QUATERNION_ZERO_EPSILON) {
            return false;
        }
        const float inv = 1.0f / n;
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;
        return true;
    }
    //! @brief Return a normalized copy
    inline Quaternion normalized() const
    {
        Quaternion q(*this);
        q.normalize();
        return q;
    }

    /*!
      @brief Rotate a vector by this quaternion (active rotation, right-handed)
      @param v Vector to rotate
      @return Rotated vector
     */
    inline Vector3 rotate(const Vector3& v) const
    {
        const Vector3 qv(x, y, z);
        const Vector3 t = qv.cross(v) * 2.0f;
        return v + t * w + qv.cross(t);
    }

    /*!
      @brief Build a rotation quaternion from an axis and angle
      @param axis Rotation axis (need not be normalized; zero axis yields identity)
      @param angle_rad Rotation angle in radians
      @return Rotation quaternion
     */
    static inline Quaternion fromAxisAngle(const Vector3& axis, const float angle_rad)
    {
        const Vector3 a  = axis.normalized();
        const float half = angle_rad * 0.5f;
        const float s    = std::sin(half);
        return Quaternion(std::cos(half), a.x * s, a.y * s, a.z * s);
    }

    /*!
      @brief Build a rotation quaternion from aerospace ZYX Euler angles
      @param roll_deg Roll (X axis) in degrees
      @param pitch_deg Pitch (Y axis) in degrees
      @param yaw_deg Yaw (Z axis) in degrees
      @return Rotation quaternion
     */
    static inline Quaternion fromEuler(const float roll_deg, const float pitch_deg, const float yaw_deg)
    {
        constexpr float deg2rad{QUATERNION_PI / 180.0f};
        const float roll  = roll_deg * deg2rad;
        const float pitch = pitch_deg * deg2rad;
        const float yaw   = yaw_deg * deg2rad;

        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);
        const float cy = std::cos(yaw * 0.5f);
        const float sy = std::sin(yaw * 0.5f);

        return Quaternion(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                          cr * cp * sy - sr * sp * cy);
    }

    /*!
      @brief Decompose this quaternion into aerospace ZYX Euler angles
      @param[out] roll_deg Roll (X axis) in degrees
      @param[out] pitch_deg Pitch (Y axis) in degrees
      @param[out] yaw_deg Yaw (Z axis) in degrees
      @details The pitch asin argument is clamped to [-1, 1] so that values near the
      gimbal lock (pitch = +-90 deg) never produce NaN.
     */
    inline void toEuler(float& roll_deg, float& pitch_deg, float& yaw_deg) const
    {
        constexpr float rad2deg{180.0f / QUATERNION_PI};

        const float sinr_cosp = 2.0f * (w * x + y * z);
        const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        const float roll      = std::atan2(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (w * y - z * x);
        if (sinp > 1.0f) {
            sinp = 1.0f;
        } else if (sinp < -1.0f) {
            sinp = -1.0f;
        }
        const float pitch = std::asin(sinp);

        const float siny_cosp = 2.0f * (w * z + x * y);
        const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        const float yaw       = std::atan2(siny_cosp, cosy_cosp);

        roll_deg  = roll * rad2deg;
        pitch_deg = pitch * rad2deg;
        yaw_deg   = yaw * rad2deg;
    }

    /*!
      @brief Integrate body-frame angular velocity over dt (gyro kinematics)
      @param gyro_rad_s Angular velocity in rad/s (body frame)
      @param dt Time step in seconds
      @details Advances this quaternion by dq = 0.5 * q (X) (0, gyro_rad_s) * dt and
      renormalizes to counteract integration drift.
     */
    inline void integrate(const Vector3& gyro_rad_s, const float dt)
    {
        const Quaternion omega(0.0f, gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z);
        const Quaternion qdot = (*this) * omega;
        const float half_dt   = 0.5f * dt;
        w += half_dt * qdot.w;
        x += half_dt * qdot.x;
        y += half_dt * qdot.y;
        z += half_dt * qdot.z;
        normalize();
    }
};

}  // namespace math
}  // namespace m5
#endif
