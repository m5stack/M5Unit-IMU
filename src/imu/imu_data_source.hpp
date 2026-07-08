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
  coefficients. User-defined sources only need to implement these three
  methods to plug into the AHRS layer.
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
};

}  // namespace imu
}  // namespace m5
#endif
