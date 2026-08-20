/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file ekf.cpp
  @brief Quaternion extended Kalman filter AHRS orientation filter
*/
#include "ekf.hpp"

#include <cmath>

using m5::math::Matrix;
using m5::math::Quaternion;
using m5::math::Vector3;

namespace m5 {
namespace imu {

namespace {
constexpr float EKF_PI{3.14159265358979323846f};
constexpr float EKF_DEG2RAD{EKF_PI / 180.0f};
constexpr float EKF_RAD2DEG{180.0f / EKF_PI};

// Initial state covariance (reset()): the quaternion starts exactly at
// identity, so a small diagonal is enough; the bias starts fully unknown, so
// its diagonal is comparatively large to let the first few observations pull
// it toward the true value quickly.
constexpr float EKF_INITIAL_QUAT_VARIANCE{1.0e-4f};
constexpr float EKF_INITIAL_BIAS_VARIANCE_DPS2{25.0f};  // (5 dps)^2

// Tuning coefficient converting the exposed "1-sigma gyro measurement noise" parameter
// into an effective continuous-time process noise density: a small per-sample sigma
// (e.g. the 0.5 dps default) understates how much the attitude estimate should stay
// responsive to slow-moving references (e.g. a leveled mag heading) once the
// covariance has shrunk from its initial value, so it is scaled up here rather than
// fed into Q as-is.
//
// Technical detail: The quaternion process noise is computed as Q_q = 0.25·ΞΞᵀ·σ²·dt,
// where the 0.25 coefficient comes from linearizing qDot = 0.5·Xi(q)·omega and is
// absorbed into this SCALE factor. The effective result is equivalent to an attitude
// random walk with σ_eff ≈ 10 dps/√s (degrees per second per root-second), which
// acts as a tuning floor that keeps the filter responsive to slow corrections.
// Numerical experiments show a valid effective window of approximately [25, 300)
// (unitless multiplicative range), beyond which convergence becomes unstable.
constexpr float EKF_PROCESS_NOISE_SCALE{100.0f};

//! @brief Symmetrize a covariance matrix in place (guards against asymmetric drift from
//! floating-point round-off in the P = F*P*F^T + Q / P = (I-KH)*P updates)
inline void symmetrize(Matrix<7, 7>& p)
{
    p = (p + p.transposed()) * 0.5f;
}

/*!
  @brief Build the 4x4 matrix Omega(omega) such that qDot = 0.5 * Omega(omega) * q
  (q as the column vector (w,x,y,z)), matching this repository's Hamilton product
  convention qDot = q (x) (0, omega) (see MadgwickFilter::update()).
 */
inline Matrix<4, 4> build_omega(const Vector3& omega)
{
    Matrix<4, 4> m{};
    m(0, 0) = 0.0f;
    m(0, 1) = -omega.x;
    m(0, 2) = -omega.y;
    m(0, 3) = -omega.z;
    m(1, 0) = omega.x;
    m(1, 1) = 0.0f;
    m(1, 2) = omega.z;
    m(1, 3) = -omega.y;
    m(2, 0) = omega.y;
    m(2, 1) = -omega.z;
    m(2, 2) = 0.0f;
    m(2, 3) = omega.x;
    m(3, 0) = omega.z;
    m(3, 1) = omega.y;
    m(3, 2) = -omega.x;
    m(3, 3) = 0.0f;
    return m;
}

/*!
  @brief Build the 4x3 matrix Xi(q) such that Omega(omega) * q == Xi(q) * omega for any
  omega, i.e. qDot = 0.5 * Xi(q) * omega. Used both to build the bias column of the state
  transition Jacobian (dqDot/db = -0.5 * Xi(q), since omega = gyro - b) and to shape the
  gyro-noise-driven process noise block of Q.
 */
inline Matrix<4, 3> build_xi(const Quaternion& q)
{
    Matrix<4, 3> m{};
    m(0, 0) = -q.x;
    m(0, 1) = -q.y;
    m(0, 2) = -q.z;
    m(1, 0) = q.w;
    m(1, 1) = -q.z;
    m(1, 2) = q.y;
    m(2, 0) = q.z;
    m(2, 1) = q.w;
    m(2, 2) = -q.x;
    m(3, 0) = -q.y;
    m(3, 1) = q.x;
    m(3, 2) = q.w;
    return m;
}

//! @brief Observation Jacobian (3x7, bias columns zero) for the predicted gravity
//! direction in the body frame, h(x) = R(q)^T * (0,0,1). Same expansion as
//! MadgwickFilter::update()'s f1/f2/f3 (verified against this file's analytic
//! derivative -- see ekf.cpp's design notes in the implementation plan).
inline Matrix<3, 7> build_h_accel(const Quaternion& q)
{
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    Matrix<3, 7> h{};
    h(0, 0) = -2.0f * y;
    h(0, 1) = 2.0f * z;
    h(0, 2) = -2.0f * w;
    h(0, 3) = 2.0f * x;
    h(1, 0) = 2.0f * x;
    h(1, 1) = 2.0f * w;
    h(1, 2) = 2.0f * z;
    h(1, 3) = 2.0f * y;
    h(2, 0) = 0.0f;
    h(2, 1) = -4.0f * x;
    h(2, 2) = -4.0f * y;
    h(2, 3) = 0.0f;
    return h;
}

//! @brief Predicted gravity direction in the body frame (h(x) for build_h_accel()).
inline Vector3 predict_accel(const Quaternion& q)
{
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    return Vector3(2.0f * (x * z - w * y), 2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
}

//! @brief Observation Jacobian (3x7, bias columns zero) for the predicted horizontal
//! magnetic field direction in the body frame, h(x) = R(q)^T * (bx,0,bz). Same
//! expansion as MadgwickFilter::update()'s f4/f5/f6 (bx, bz frozen at their per-step
//! recomputed value, not differentiated w.r.t. q -- same approximation Madgwick uses).
inline Matrix<3, 7> build_h_mag(const Quaternion& q, const float bx, const float bz)
{
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;

    Matrix<3, 7> h{};
    h(0, 0) = -2.0f * bz * y;
    h(0, 1) = 2.0f * bz * z;
    h(0, 2) = -4.0f * bx * y - 2.0f * bz * w;
    h(0, 3) = -4.0f * bx * z + 2.0f * bz * x;
    h(1, 0) = -2.0f * bx * z + 2.0f * bz * x;
    h(1, 1) = 2.0f * bx * y + 2.0f * bz * w;
    h(1, 2) = 2.0f * bx * x + 2.0f * bz * z;
    h(1, 3) = -2.0f * bx * w + 2.0f * bz * y;
    h(2, 0) = 2.0f * bx * y;
    h(2, 1) = 2.0f * bx * z - 4.0f * bz * x;
    h(2, 2) = 2.0f * bx * w - 4.0f * bz * y;
    h(2, 3) = 2.0f * bx * x;
    return h;
}

//! @brief Predicted horizontal-mag-reference direction in the body frame (h(x) for
//! build_h_mag()).
inline Vector3 predict_mag(const Quaternion& q, const float bx, const float bz)
{
    const float w = q.w;
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    return Vector3(bx * (1.0f - 2.0f * (y * y + z * z)) + 2.0f * bz * (x * z - w * y),
                   2.0f * bx * (x * y - w * z) + 2.0f * bz * (y * z + w * x),
                   2.0f * bx * (x * z + w * y) + bz * (1.0f - 2.0f * (x * x + y * y)));
}

}  // namespace

void EKFFilter::reset_state()
{
    _q          = Quaternion::identity();
    _bias_rad_s = Vector3();
    _p          = Matrix<7, 7>();
    for (int i = 0; i < 4; ++i) {
        _p(i, i) = EKF_INITIAL_QUAT_VARIANCE;
    }
    const float bias_var_rad2 = EKF_INITIAL_BIAS_VARIANCE_DPS2 * EKF_DEG2RAD * EKF_DEG2RAD;
    for (int i = 4; i < 7; ++i) {
        _p(i, i) = bias_var_rad2;
    }
}

void EKFFilter::reset()
{
    reset_state();
}

void EKFFilter::predict(const float dt, const Vector3& gyro_rad_s)
{
    const Vector3 omega = gyro_rad_s - _bias_rad_s;

    // Jacobians are linearized at the pre-propagation estimate.
    const Matrix<4, 4> omega_mat = build_omega(omega);
    const Matrix<4, 3> xi        = build_xi(_q);

    Matrix<7, 7> f{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            f(r, c) = (r == c ? 1.0f : 0.0f) + dt * 0.5f * omega_mat(r, c);
        }
        for (int c = 0; c < 3; ++c) {
            f(r, 4 + c) = -0.5f * dt * xi(r, c);
        }
    }
    for (int i = 4; i < 7; ++i) {
        f(i, i) = 1.0f;
    }

    const float gyro_var_rad2s2 = (_gyro_noise_dps * EKF_DEG2RAD) * (_gyro_noise_dps * EKF_DEG2RAD);
    const float bias_rw_var     = (_bias_random_walk_dps * EKF_DEG2RAD) * (_bias_random_walk_dps * EKF_DEG2RAD);
    const Matrix<4, 4> xi_xit   = xi * xi.transposed();

    // Angular random walk driven by the gyro noise, integrated over dt (variance grows
    // linearly with dt, as for a continuous white-noise-driven process -- not dt^2, which
    // would make the quaternion covariance collapse towards zero far too quickly and stall
    // corrections from slow-moving reference directions such as a leveled mag heading).
    Matrix<7, 7> q{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            q(r, c) = xi_xit(r, c) * gyro_var_rad2s2 * dt * EKF_PROCESS_NOISE_SCALE;
        }
    }
    for (int i = 4; i < 7; ++i) {
        q(i, i) = bias_rw_var * dt;
    }

    _q.integrate(omega, dt);  // Bias is held constant in the process model.
    _p = f * _p * f.transposed() + q;
    symmetrize(_p);
}

bool EKFFilter::apply_axis3_update(const Matrix<3, 7>& h, const Vector3& innovation, const float noise_var)
{
    const Matrix<7, 3> ht = h.transposed();
    const Matrix<3, 7> hp = h * _p;
    const Matrix<3, 3> s  = hp * ht + Matrix<3, 3>::identity() * noise_var;

    Matrix<3, 3> s_inv;
    if (!m5::math::invert3x3(s, s_inv)) {
        return false;  // Skip this observation rather than corrupt the state.
    }

    const Matrix<7, 3> pht = _p * ht;
    const Matrix<7, 3> k   = pht * s_inv;

    Matrix<3, 1> innovation_col;
    innovation_col(0, 0)  = innovation.x;
    innovation_col(1, 0)  = innovation.y;
    innovation_col(2, 0)  = innovation.z;
    const Matrix<7, 1> dx = k * innovation_col;

    _q.w += dx(0, 0);
    _q.x += dx(1, 0);
    _q.y += dx(2, 0);
    _q.z += dx(3, 0);
    _q.normalize();
    _bias_rad_s.x += dx(4, 0);
    _bias_rad_s.y += dx(5, 0);
    _bias_rad_s.z += dx(6, 0);

    const Matrix<7, 7> kh = k * h;
    _p                    = (Matrix<7, 7>::identity() - kh) * _p;
    symmetrize(_p);
    return true;
}

void EKFFilter::update(const float dt, const float accel[3], const float gyro[3], const float* mag)
{
    if (dt <= 0.0f) {
        return;
    }

    const Vector3 gyro_rad_s(gyro[0] * EKF_DEG2RAD, gyro[1] * EKF_DEG2RAD, gyro[2] * EKF_DEG2RAD);
    predict(dt, gyro_rad_s);

    const float a_norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (a_norm > 0.0f) {
        const Vector3 a_measured(accel[0] / a_norm, accel[1] / a_norm, accel[2] / a_norm);
        const Vector3 a_predicted = predict_accel(_q);
        const Vector3 innovation  = a_measured - a_predicted;
        const Matrix<3, 7> h      = build_h_accel(_q);
        const float noise         = _accel_noise_g / a_norm;
        apply_axis3_update(h, innovation, noise * noise);
    }

    if (mag != nullptr) {
        const float m_norm = std::sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
        if (m_norm > 0.0f) {
            const Vector3 m_measured(mag[0] / m_norm, mag[1] / m_norm, mag[2] / m_norm);

            // Rotate the measured (normalized) mag reading body->world with the current
            // attitude estimate, then flatten its horizontal component onto world +X to
            // get this step's world-frame mag reference (bx, 0, bz) -- recomputed every
            // step, per Madgwick 2010 (see MadgwickFilter::update()/MahonyFilter::update()).
            const Vector3 h_rot = _q.rotate(m_measured);
            const float bx      = std::sqrt(h_rot.x * h_rot.x + h_rot.y * h_rot.y);
            const float bz      = h_rot.z;

            const Vector3 m_predicted = predict_mag(_q, bx, bz);
            const Vector3 innovation  = m_measured - m_predicted;
            const Matrix<3, 7> h      = build_h_mag(_q, bx, bz);
            const float noise         = _mag_noise_ut / m_norm;
            apply_axis3_update(h, innovation, noise * noise);
        }
    }
}

void EKFFilter::quaternion(float& w, float& x, float& y, float& z) const
{
    w = _q.w;
    x = _q.x;
    y = _q.y;
    z = _q.z;
}

bool EKFFilter::writeNoiseParams(const float gyro_noise_dps, const float accel_noise_g, const float mag_noise_ut,
                                 const float bias_random_walk_dps)
{
    if (!(gyro_noise_dps > 0.0f) || !(accel_noise_g > 0.0f) || !(mag_noise_ut > 0.0f) ||
        !(bias_random_walk_dps > 0.0f)) {
        return false;
    }
    _gyro_noise_dps       = gyro_noise_dps;
    _accel_noise_g        = accel_noise_g;
    _mag_noise_ut         = mag_noise_ut;
    _bias_random_walk_dps = bias_random_walk_dps;
    return true;
}

Vector3 EKFFilter::readGyroBias() const
{
    return Vector3(_bias_rad_s.x * EKF_RAD2DEG, _bias_rad_s.y * EKF_RAD2DEG, _bias_rad_s.z * EKF_RAD2DEG);
}

bool EKFFilter::writeQuaternion(const float w, const float x, const float y, const float z)
{
    Quaternion q(w, x, y, z);
    if (!q.normalize()) {
        return false;
    }
    _q = q;
    return true;
}

}  // namespace imu
}  // namespace m5
