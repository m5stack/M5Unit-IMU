/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file ahrs_imu.hpp
  @brief M5Unified-compatible facade over AhrsController
  @details AhrsImu derives from AhrsController and adds thin forwarders that mirror
  m5::IMU_Class method names, signatures and data layout, so code written against
  M5Unified's M5.Imu ports here with minimal churn. All M5Unified vocabulary (the get/set
  spelling and the imu_data_t/imu_3d_t types) is confined to this file; AhrsController
  itself stays free of any M5Unified concept and keeps this library's write/read
  convention. This header pulls in no M5Unified source -- the forwarders are built purely
  on AhrsController's public API (latest(), euler(), writeAutoCalibration(), and the
  hasAccel()/hasGyro()/hasTemp()/hasValidMag()/lastMicros() queries).
*/
#ifndef M5_UNIT_IMU_IMU_AHRS_IMU_HPP
#define M5_UNIT_IMU_IMU_AHRS_IMU_HPP

#include <cstdint>

#include "ahrs_controller.hpp"

namespace m5 {
namespace imu {

/*!
  @class AhrsImu
  @brief AhrsController plus an M5Unified IMU_Class-compatible API
  @details Use this in place of AhrsController when porting M5Unified M5.Imu code: it is-a
  AhrsController (so it plugs into presets, add(), selectFilter(), etc. unchanged) and
  additionally exposes getAccel/getGyro/getMag/getTemp/getImuData/getAhrsData/setCalibration.
  The getters read the already axis-corrected, offset-compensated latest() sample; setCalibration
  forwards to writeAutoCalibration(); getAhrsData() provides the attitude output for the
  getAhrsData() slot that M5Unified's IMU_Class declares but does not implement.
 */
class AhrsImu : public AhrsController {
public:
    /*!
      @struct imu_3d_t
      @brief 3-axis float vector, layout-compatible with m5::IMU_Class::imu_3d_t
     */
    struct imu_3d_t {
        union {
            float value[3];
            struct {
                float x;
                float y;
                float z;
            };
        };
    };

    /*!
      @struct imu_data_t
      @brief Merged accel/gyro/mag sample, layout-compatible with m5::IMU_Class::imu_data_t
     */
    struct imu_data_t {
        uint32_t usec;  //!< @brief Timestamp of the sample's update() call (microseconds)
        union {
            float value[9];
            imu_3d_t sensor[3];
            struct {
                imu_3d_t accel;
                imu_3d_t gyro;
                imu_3d_t mag;
            };
        };
    };

    /*!
      @enum axis_t
      @brief A source axis with a sign, layout-compatible with m5::IMU_Class::axis_t
      @details Bit 0 is the sign (1 = negated), bits 1-2 select the axis (0 = X, 1 = Y, 2 = Z).
     */
    enum axis_t {
        axis_x_pos = 0,  //!< @brief +X
        axis_x_neg = 1,  //!< @brief -X
        axis_y_pos = 2,  //!< @brief +Y
        axis_y_neg = 3,  //!< @brief -Y
        axis_z_pos = 4,  //!< @brief +Z
        axis_z_neg = 5,  //!< @brief -Z
    };

    /*!
      @brief Latest accelerometer reading (G)
      @param[out] ax Accelerometer X
      @param[out] ay Accelerometer Y
      @param[out] az Accelerometer Z
      @return True if an accelerometer source is registered
     */
    inline bool getAccel(float* const ax, float* const ay, float* const az) const
    {
        const imu_physical_data_t& s = latest();
        if (ax != nullptr) {
            *ax = s.accel[0];
        }
        if (ay != nullptr) {
            *ay = s.accel[1];
        }
        if (az != nullptr) {
            *az = s.accel[2];
        }
        return hasAccel();
    }

    /*!
      @brief Latest gyroscope reading (dps)
      @param[out] gx Gyroscope X
      @param[out] gy Gyroscope Y
      @param[out] gz Gyroscope Z
      @return True if a gyroscope source is registered
     */
    inline bool getGyro(float* const gx, float* const gy, float* const gz) const
    {
        const imu_physical_data_t& s = latest();
        if (gx != nullptr) {
            *gx = s.gyro[0];
        }
        if (gy != nullptr) {
            *gy = s.gyro[1];
        }
        if (gz != nullptr) {
            *gz = s.gyro[2];
        }
        return hasGyro();
    }

    /*!
      @brief Latest magnetometer reading (uT)
      @param[out] mx Magnetometer X
      @param[out] my Magnetometer Y
      @param[out] mz Magnetometer Z
      @return True if at least one valid magnetometer sample has been seen; false on a 6-axis setup
     */
    inline bool getMag(float* const mx, float* const my, float* const mz) const
    {
        const imu_physical_data_t& s = latest();
        if (mx != nullptr) {
            *mx = s.mag[0];
        }
        if (my != nullptr) {
            *my = s.mag[1];
        }
        if (mz != nullptr) {
            *mz = s.mag[2];
        }
        return hasValidMag();
    }

    /*!
      @brief Latest temperature reading (degC)
      @param[out] t Temperature
      @return True if a temperature source is registered
     */
    inline bool getTemp(float* const t) const
    {
        if (t != nullptr) {
            *t = latest().temp;
        }
        return hasTemp();
    }

    /*!
      @brief Copy the latest accel/gyro/mag sample into an imu_data_t
      @param[out] data Destination (ignored if nullptr)
      @details usec carries lastMicros(), i.e. the timestamp of the most recent self-clocked
      update(); it stays 0 when the controller is driven by update(const float dt) instead.
     */
    inline void getImuData(imu_data_t* const data) const
    {
        if (data == nullptr) {
            return;
        }
        const imu_physical_data_t& s = latest();
        data->usec                   = static_cast<uint32_t>(lastMicros());
        data->accel.x                = s.accel[0];
        data->accel.y                = s.accel[1];
        data->accel.z                = s.accel[2];
        data->gyro.x                 = s.gyro[0];
        data->gyro.y                 = s.gyro[1];
        data->gyro.z                 = s.gyro[2];
        data->mag.x                  = s.mag[0];
        data->mag.y                  = s.mag[1];
        data->mag.z                  = s.mag[2];
    }

    /*!
      @brief Latest accel/gyro/mag sample, by const reference
      @return Reference to an internal imu_data_t refreshed on each call
     */
    inline const imu_data_t& getImuData()
    {
        getImuData(&_compat_data);
        return _compat_data;
    }

    /*!
      @brief Current fused attitude as aerospace Euler angles (degrees)
      @param[out] pitch Pitch (Y axis)
      @param[out] roll Roll (X axis)
      @param[out] yaw Yaw (Z axis)
      @details Forwards to euler(). Provides the getAhrsData() attitude output that
      M5Unified's IMU_Class declares but does not implement. Argument order matches
      M5Unified's planned signature (pitch, roll, yaw).
     */
    inline void getAhrsData(float* const pitch, float* const roll, float* const yaw) const
    {
        float roll_deg{}, pitch_deg{}, yaw_deg{};
        euler(roll_deg, pitch_deg, yaw_deg);
        if (pitch != nullptr) {
            *pitch = pitch_deg;
        }
        if (roll != nullptr) {
            *roll = roll_deg;
        }
        if (yaw != nullptr) {
            *yaw = yaw_deg;
        }
    }

    //! @brief Alias for getAccel()
    inline bool getAccelData(float* const ax, float* const ay, float* const az) const
    {
        return getAccel(ax, ay, az);
    }

    //! @brief Alias for getGyro()
    inline bool getGyroData(float* const gx, float* const gy, float* const gz) const
    {
        return getGyro(gx, gy, gz);
    }

    //! @brief Alias for getMag() (name mirrors m5::IMU_Class::getGyroMag)
    inline bool getGyroMag(float* const mx, float* const my, float* const mz) const
    {
        return getMag(mx, my, mz);
    }

    /*!
      @brief Whether accelerometer and gyroscope sources are both registered
      @return True once begin() has an accel+gyro source, mirroring m5::IMU_Class::isEnabled()
     */
    inline bool isEnabled() const
    {
        return hasAccel() && hasGyro();
    }

    /*!
      @brief Set per-capability auto-calibration strength (0 = off, 1-255 = adjustment speed)
      @param accel_strength Accelerometer strength
      @param gyro_strength Gyroscope strength
      @param mag_strength Magnetometer strength
      @details Alias for writeAutoCalibration().
     */
    inline void setCalibration(const uint8_t accel_strength, const uint8_t gyro_strength,
                               const uint8_t mag_strength)
    {
        writeAutoCalibration(accel_strength, gyro_strength, mag_strength);
    }

    /*!
      @brief Zero every capability's auto-calibration offset
      @details Forwards to writeCalibration() with a default (all-zero) snapshot, mirroring
      m5::IMU_Class::clearOffsetData(). Auto-calibration strengths set via setCalibration()
      are left unchanged; only the accumulated offsets are cleared.
     */
    inline void clearOffsetData()
    {
        writeCalibration(calibration_data_t{});
    }

    /*!
      @brief Assign which signed source axis feeds each output slot
      @param axis0 Source axis (with sign) for output X
      @param axis1 Source axis (with sign) for output Y
      @param axis2 Source axis (with sign) for output Z
      @return False (and no change) if the three axes are not a permutation of X/Y/Z
      @details Mirrors m5::IMU_Class::setAxisOrder. Converts the axis_t triplet to an AxisOrder
      and installs it as the global user remap via writeUserAxisOrder(), so it composes on top
      of every source's fixed board correction and applies to accel/gyro/mag alike.
     */
    inline bool setAxisOrder(const axis_t axis0, const axis_t axis1, const axis_t axis2)
    {
        const uint8_t idx[3] = {
            static_cast<uint8_t>(axis0 >> 1),
            static_cast<uint8_t>(axis1 >> 1),
            static_cast<uint8_t>(axis2 >> 1),
        };
        if (idx[0] > 2 || idx[1] > 2 || idx[2] > 2 || idx[0] == idx[1] || idx[1] == idx[2] ||
            idx[0] == idx[2]) {
            return false;
        }
        AxisOrder order;
        order.permutation = permutation_from_indices(idx);
        order.invert_x    = (axis0 & 1) != 0;
        order.invert_y    = (axis1 & 1) != 0;
        order.invert_z    = (axis2 & 1) != 0;
        writeUserAxisOrder(order);
        return true;
    }

    /*!
      @brief Assign the first two output axes; derive the third as a right-handed frame
      @param axis0 Source axis (with sign) for output X
      @param axis1 Source axis (with sign) for output Y
      @return False (and no change) if axis0 and axis1 share the same axis
      @details Mirrors m5::IMU_Class::setAxisOrderRightHanded.
     */
    inline bool setAxisOrderRightHanded(const axis_t axis0, const axis_t axis1)
    {
        return setAxisOrder(axis0, axis1, getAxis2(axis0, axis1));
    }

    /*!
      @brief Assign the first two output axes; derive the third as a left-handed frame
      @param axis0 Source axis (with sign) for output X
      @param axis1 Source axis (with sign) for output Y
      @return False (and no change) if axis0 and axis1 share the same axis
      @details Mirrors m5::IMU_Class::setAxisOrderLeftHanded (the right-handed third axis with
      its sign flipped).
     */
    inline bool setAxisOrderLeftHanded(const axis_t axis0, const axis_t axis1)
    {
        return setAxisOrder(axis0, axis1, static_cast<axis_t>(getAxis2(axis0, axis1) ^ 1));
    }

private:
    // Right-handed third axis for a given first two (ported from m5::IMU_Class getAxis2)
    static inline axis_t getAxis2(const axis_t axis0, const axis_t axis1)
    {
        uint8_t axis2   = static_cast<uint8_t>(((1 << (axis0 >> 1)) | (1 << (axis1 >> 1))) ^ 0x07);
        axis2           = static_cast<uint8_t>(axis2 & 0x06);
        const uint8_t ax0 = axis2 ? static_cast<uint8_t>(axis2 - 2) : 4;
        axis2 = static_cast<uint8_t>(axis2 + (((axis0 & 0x06) == ax0) == ((axis0 & 1) == (axis1 & 1))));
        return static_cast<axis_t>(axis2);
    }

    imu_data_t _compat_data{};  // Scratch for the reference-returning getImuData() overload
};

}  // namespace imu
}  // namespace m5
#endif
