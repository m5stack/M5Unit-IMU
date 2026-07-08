/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file ahrs_controller.hpp
  @brief Layer-2 AHRS controller: merges IMUDataSource capabilities and drives an AhrsFilterBase
*/
#ifndef M5_UNIT_IMU_IMU_AHRS_CONTROLLER_HPP
#define M5_UNIT_IMU_IMU_AHRS_CONTROLLER_HPP

#include <cstddef>
#include <cstdint>

#include "filter/filter_base.hpp"
#include "imu_data_source.hpp"
#include "imu_types.hpp"

namespace m5 {
namespace imu {

/*!
  @class AhrsController
  @brief Merges up to 4 IMUDataSource instances by capability and fuses their samples into
  an attitude estimate via a pluggable AhrsFilterBase
  @details
  - **Capability merge**: add() assigns each of Accel/Gyro/Mag/Temp to the first source
    that reports it ("first come, first served"); later sources reporting an
    already-claimed capability are still drained every update() (so their internal ring
    buffers do not overflow) but their samples for that capability are discarded.
  - **Primary clock**: the gyro-owning source drives the filter. Every update() call
    drains the gyro source fully; each popped gyro sample triggers one
    `AhrsFilterBase::update()` call, with the elapsed time divided evenly across the
    batch (`dt / <samples drained this call>`). Accel/mag samples from a *different*
    source than the gyro are drained once per update() call and the resulting "latest"
    value is reused for every filter update() call in that batch (no per-sample
    interpolation across sources); a source shared with the gyro (e.g. a single 6/9-axis
    chip) is synchronized sample-for-sample instead.
  - **Magnetometer overflow**: a mag sample where any axis reads exactly -32768 (the
    BMM150 OVERFLOW sentinel) is rejected outright; the previous valid mag value is kept
    (both for latest() and for whatever gets passed to the filter). If no valid mag
    sample has ever been seen, the filter is fed `mag == nullptr` (6-axis operation).
  - **Clocking**: update() measures dt itself via m5::utility::micros() and smooths it
    with an exponential moving average (see ahrs_controller.cpp) before handing it to the
    filter, to avoid feeding raw, jittery sample-to-sample deltas straight into filters
    that resample their internal state on large dt swings (see vqf.hpp). update(const
    float dt) bypasses both the clock and the smoothing -- the caller's dt is used as-is
    for the whole batch drained during that call, which is what test code (and any other
    caller with its own timebase) should use.
 */
class AhrsController {
public:
    AhrsController() = default;
    ~AhrsController();

    AhrsController(const AhrsController&) = delete;
    AhrsController& operator=(const AhrsController&) = delete;

    /*!
      @brief Register an IMU data source
      @param src Source to add (must outlive this controller)
      @return False if this controller already holds the maximum number of sources
      @details Never fails due to a capability already being claimed by another source --
      duplicate capabilities are simply ignored (the first source to report them wins).
     */
    bool add(IMUDataSource& src);

    /*!
      @brief Prepare this controller for update()
      @return False if no source has claimed both Accel and Gyro
      @details Selects FilterType::Madgwick if no filter has been chosen yet (see
      selectFilter()/setFilter()).
     */
    bool begin();

    /*!
      @brief Drain every source and fuse newly-arrived samples, measuring dt from the
      real clock (m5::utility::micros())
      @return Capability bits that received at least one new sample this call
      @details The very first call after construction only seeds the internal clock (no
      previous timestamp to compute an elapsed time from) and returns Capability::None
      without draining anything; every following call computes dt from the elapsed time
      since the previous call.
     */
    Capability update();

    /*!
      @brief Drain every source and fuse newly-arrived samples using an explicit dt
      @param dt Time elapsed since the previous update call, in seconds; divided evenly
      across however many gyro samples are drained this call
      @return Capability bits that received at least one new sample this call
     */
    Capability update(const float dt);

    /*!
      @brief Select and internally construct one of the built-in filters
      @param type Filter to construct
      @return True on success
      @details Replaces (and resets) whatever filter is currently active, deleting a
      previously internally-constructed filter first. Does not delete a filter injected
      via setFilter().
     */
    bool selectFilter(const FilterType type);

    /*!
      @brief Inject an externally-owned filter
      @param filter Filter instance to use from now on (must outlive this controller)
      @details Deletes a previously internally-constructed filter (selectFilter()), if
      any. Ownership of `filter` itself is not taken.
     */
    void setFilter(AhrsFilterBase& filter);

    //! @brief Currently active filter, or nullptr if none has been selected yet
    inline AhrsFilterBase* filter() const
    {
        return _filter;
    }

    /*!
      @brief Current fused attitude
      @param[out] w Quaternion scalar part
      @param[out] x Quaternion vector part X
      @param[out] y Quaternion vector part Y
      @param[out] z Quaternion vector part Z
      @details Yields the identity quaternion if no filter has been selected/injected yet.
     */
    void quaternion(float& w, float& x, float& y, float& z) const;

    /*!
      @brief Current fused attitude as aerospace ZYX Euler angles
      @param[out] roll_deg Roll (X axis), degrees
      @param[out] pitch_deg Pitch (Y axis), degrees
      @param[out] yaw_deg Yaw (Z axis), degrees
     */
    void euler(float& roll_deg, float& pitch_deg, float& yaw_deg) const;

    //! @brief Most recently fused physical-unit sample
    inline const imu_physical_data_t& latest() const
    {
        return _latest;
    }

private:
    static constexpr size_t MAX_SOURCES{4};

    static bool is_mag_overflow(const int32_t mag[3]);
    Capability drain_and_fuse(const float dt);

    IMUDataSource* _sources[MAX_SOURCES]{};
    size_t _source_count{};

    IMUDataSource* _accel_src{};
    IMUDataSource* _gyro_src{};
    IMUDataSource* _mag_src{};
    IMUDataSource* _temp_src{};

    AhrsFilterBase* _owned_filter{};  // Non-null only when constructed by selectFilter()
    AhrsFilterBase* _filter{};        // Active filter (owned or injected); nullptr until chosen

    imu_physical_data_t _latest{};
    bool _has_valid_mag{false};

    unsigned long _last_micros{};
    bool _has_last_micros{false};
    float _dt_ema{};
    bool _has_dt_ema{false};
};

}  // namespace imu
}  // namespace m5
#endif
