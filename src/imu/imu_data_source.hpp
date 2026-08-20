/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file imu_data_source.hpp
  @brief Public interface for IMU data providers
*/
#ifndef M5_UNIT_IMU_IMU_IMU_DATA_SOURCE_HPP
#define M5_UNIT_IMU_IMU_IMU_DATA_SOURCE_HPP

#include "imu_types.hpp"

namespace m5 {
namespace imu {

/*!
  @class IMUDataSource
  @brief Thin contract between measurement units and consumers (e.g. AHRS)
  @details Implementations report their capability, hand out unconsumed raw
  samples in oldest-first order (pop semantics), and expose conversion
  coefficients. User-defined sources only need to implement those three
  methods to plug into the AHRS layer; sleep()/wakeup() are optional and
  default to doing nothing.
 */
class IMUDataSource {
public:
    virtual ~IMUDataSource() = default;

    /*!
      @brief What this source can provide
      @return Capability bits supported by this source
     */
    virtual Capability imuSpec() const = 0;
    /*!
      @brief Pop the oldest unconsumed raw sample
      @param[out] out Sample destination
      @return Items actually filled (Capability::None if nothing left)
     */
    virtual Capability readImuRaw(imu_raw_data_t& out) = 0;
    /*!
      @brief Conversion coefficients for this source
      @param[out] out Conversion coefficients destination
     */
    virtual void imuConvertParam(imu_convert_param_t& out) const = 0;

    /*!
      @brief Put the device into its lowest reversible power state
      @return True if this source implements sleeping and the device accepted it
      @details Optional: the default does nothing and returns false, so sources that cannot be
      suspended stay safe to aggregate. Implementations leave the device producing no samples
      while asleep. Periodic measurement is stopped; wakeup() does not restore it and callers
      must invoke startPeriodicMeasurement() again to resume sampling.
     */
    virtual bool sleep()
    {
        return false;
    }
    /*!
      @brief Bring the device back from sleep()
      @return True if this source implements waking and the device accepted it
      @details Optional: the default does nothing and returns false. Implementations bring the
      device back to a configurable state; they do not restore the measurement mode that was
      active before sleep(). Callers must invoke startPeriodicMeasurement() again to resume
      sampling. Calling this without a preceding sleep() must be harmless.
     */
    virtual bool wakeup()
    {
        return false;
    }
};

}  // namespace imu
}  // namespace m5
#endif
