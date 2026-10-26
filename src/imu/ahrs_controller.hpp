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

#include <math/quaternion.hpp>

#include "axis_order.hpp"
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
    batch (`dt / <samples drained this call>`). The time of a call that drains no gyro
    sample is carried over to the next call that does, so calling update() faster than the
    gyro ODR does not shrink the integrated rotation. Accel/mag samples from a *different*
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
    for the whole batch drained during that call, which is what callers with their own
    timebase should use.
 */
class AhrsController {
public:
    /*!
      @struct calibration_data_t
      @brief Exportable/importable snapshot of every capability's auto-calibration offset
      @details POD, versioned so a future layout change can be detected on import (see
      readCalibration()/writeCalibration()).
     */
    struct calibration_data_t {
        uint32_t version{1};      //!< @brief Struct layout version
        float accel_offset[3]{};  //!< @brief Accelerometer offset (G), X/Y/Z
        float gyro_offset[3]{};   //!< @brief Gyroscope offset (dps), X/Y/Z
        float mag_offset[3]{};    //!< @brief Magnetometer offset (uT), X/Y/Z
    };

    //! @brief Construct with no sources and the default filter (Complementary)
    AhrsController();
    //! @brief Destructor
    ~AhrsController();

    //! @brief Copy construction is disabled (holds pointers to registered sources)
    AhrsController(const AhrsController&) = delete;
    //! @brief Copy assignment is disabled (holds pointers to registered sources)
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
      @brief Set the chip-frame-to-body-frame axis remap applied to one source's Accel/Gyro
      (and Temp) samples
      @param src Source to configure (must already have been add()ed)
      @param order Permutation and per-axis sign flip to apply
      @return False if src was never add()ed to this controller
      @details Applied every update() to whichever of Accel/Gyro/Temp that source provides,
      right after raw-to-physical conversion and before auto-calibration offsets are
      subtracted (see writeAutoCalibration()). Does **not** affect Mag -- accel and gyro
      always share a single die and are therefore always co-oriented, but an auxiliary
      magnetometer die (e.g. a BMM150 mounted as a BMI270 AUX chip) can be mounted at a
      different orientation on the very same source; use writeMagAxisOrder() for that.
      Defaults to the identity transform (AxisPermutation::XYZ, no inversion) for every
      added source.
     */
    bool writeAxisOrder(const IMUDataSource& src, const AxisOrder& order);

    /*!
      @brief Read back the Accel/Gyro axis remap currently configured for one source
      @param src Source to query (must already have been add()ed)
      @param[out] out Destination for the current AxisOrder
      @return False (out left untouched) if src was never add()ed to this controller
     */
    bool readAxisOrder(const IMUDataSource& src, AxisOrder& out) const;

    /*!
      @brief Set the chip-frame-to-body-frame axis remap applied to one source's Mag samples
      @param src Source to configure (must already have been add()ed)
      @param order Permutation and per-axis sign flip to apply to Mag only
      @return False if src was never add()ed to this controller
      @details Applied every update() to the Mag capability of whichever source provides it,
      right after raw-to-physical conversion and before auto-calibration offsets are
      subtracted (see writeAutoCalibration()) -- independent of writeAxisOrder(), which only
      covers Accel/Gyro/Temp. Defaults to the identity transform (AxisPermutation::XYZ, no
      inversion) for every added source.
     */
    bool writeMagAxisOrder(const IMUDataSource& src, const AxisOrder& order);

    /*!
      @brief Read back the Mag-only axis remap currently configured for one source
      @param src Source to query (must already have been add()ed)
      @param[out] out Destination for the current AxisOrder
      @return False (out left untouched) if src was never add()ed to this controller
     */
    bool readMagAxisOrder(const IMUDataSource& src, AxisOrder& out) const;

    /*!
      @brief Set a global user axis remap composed on top of every source's fixed order
      @param order Permutation and per-axis sign flip the caller wants applied to the reported frame
      @details Mirrors M5Unified's IMU_Class::setAxisOrder user layer. writeAxisOrder()/
      writeMagAxisOrder() carry the fixed, per-board correction (chip frame to a unified body
      frame); this order is applied *after* that, reorienting the whole device output frame, and
      so it affects Accel, Gyro, Temp and Mag of every source alike. The two are folded together
      internally (see compose() in axis_order.hpp), so no extra per-sample transform is paid.
      Defaults to the identity transform. Composition is recomputed here and on every
      writeAxisOrder()/writeMagAxisOrder()/add().
     */
    void writeUserAxisOrder(const AxisOrder& order);

    /*!
      @brief Read back the current global user axis remap
      @param[out] out Destination for the current user AxisOrder (identity by default)
     */
    void readUserAxisOrder(AxisOrder& out) const;

    /*!
      @brief Enable/disable continuous background auto-calibration per capability
      @param accel Accelerometer strength, 0 = off, 1-255 = adjustment speed (higher = faster)
      @param gyro Gyroscope strength, 0 = off, 1-255 = adjustment speed
      @param mag Magnetometer strength, 0 = off, 1-255 = adjustment speed
      @note Only the gyroscope is on by default (strength 1); accel and mag start at 0. An
      uncorrected gyro zero-rate offset is the dominant source of yaw drift and, when it exceeds
      VQFFilter::writeBiasClip()'s window, also stops VQF from estimating the bias on its own --
      so the default keeps it in check without any setup. Strength is effectively the per-sample
      follow rate x255 while the device is still, so 1 already converges with a roughly 1.3s time
      constant at a 200Hz sample rate.
      @details A stillness/sphere-fit auto-offset algorithm operating on physical-unit
      samples. Each capability tracks how "still" its recent samples
      have been; once that stillness score exceeds a threshold, the offset is nudged toward
      fitting the sample to the expected shape at rest (accel/mag: a sphere of a fixed
      radius around the offset; gyro: the origin). The resulting offset is subtracted from
      every subsequent sample of that capability regardless of strength -- setting strength
      to 0 only stops further adjustment, it does not clear a previously-converged offset
      (see writeCalibration()/clearing via calibration_data_t{}).
     */
    void writeAutoCalibration(const uint8_t accel, const uint8_t gyro, const uint8_t mag);

    /*!
      @brief Read back the current auto-calibration strengths
      @param[out] accel Accelerometer strength
      @param[out] gyro Gyroscope strength
      @param[out] mag Magnetometer strength
     */
    void readAutoCalibration(uint8_t& accel, uint8_t& gyro, uint8_t& mag) const;

    /*!
      @brief Pin the field strength the magnetometer auto-calibration fits its samples to
      @param radius_ut Local geomagnetic field strength in uT; pass 0 (or less) to go back to
      estimating it from the data, which is the default
      @details The hard-iron fit works by pushing the offset until every sample sits on a sphere
      of this radius, so the radius has to match the field the device is actually in. Unlike the
      accelerometer's 1G sphere, that is not a universal constant: the geomagnetic field runs
      from roughly 25uT to 65uT depending on location, and a mismatched radius mis-scales the
      calibration rather than correcting it. By default the radius is therefore learned from the
      samples themselves; set it here when the local value is known (national geomagnetic surveys
      publish it) to skip the learning transient.
      @note Changing this resets the learned estimate, not the offset itself; clear the offset via
      writeCalibration() if a previous calibration converged against a different radius.
     */
    void writeMagCalibrationRadius(const float radius_ut);

    /*!
      @brief Field strength the magnetometer auto-calibration is currently fitting to
      @return Radius in uT; 0 while the estimate is still unseeded (no calibration has run yet)
     */
    float readMagCalibrationRadius() const;

    /*!
      @brief Collect gyro samples for the given duration (real clock) and set the gyro
      offset to their mean
      @param duration_ms Collection duration, milliseconds
      @return False if no source has claimed Gyro, or no sample was drained during that time
      @details Intended for hardware use: repeatedly calls update() (real clock, via
      m5::utility::millis()) for approximately duration_ms while the unit is held still,
      then overwrites the gyro offset with the mean of every gyro sample seen. See
      calibrateGyro(const uint32_t, const float) for the deterministic, explicit-dt sibling
      (drives the same sample collection through update(const float dt) instead of the real
      clock).
     */
    bool calibrateGyro(const uint32_t duration_ms);

    /*!
      @brief Deterministic sibling of calibrateGyro(const uint32_t): explicit dt instead of
      the real clock
      @param update_count Number of update(dt) calls to run while collecting
      @param dt Time step passed to each update(dt) call, seconds
      @return False if no source has claimed Gyro, or no sample was drained during that time
     */
    bool calibrateGyro(const uint32_t update_count, const float dt);

    /*!
      @brief Snapshot every capability's current auto-calibration offset
      @param[out] out Destination for the snapshot
      @return Always true
     */
    bool readCalibration(calibration_data_t& out) const;

    /*!
      @brief Overwrite every capability's auto-calibration offset from a snapshot
      @param data Snapshot to restore (see readCalibration())
      @return False (offsets left untouched) if data.version is not the currently supported version
     */
    bool writeCalibration(const calibration_data_t& data);

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
      across however many gyro samples are drained this call (if none is drained, it is
      carried over to the next call that drains one)
      @return Capability bits that received at least one new sample this call
     */
    Capability update(const float dt);

    /*!
      @brief Put every added source into its lowest reversible power state
      @return True if at least one source actually slept
      @details Calls IMUDataSource::sleep() on all sources, including those whose capability
      was not claimed. Sources that do not implement sleeping report false and are skipped, so
      a false return only means nothing could be suspended. Sleeping stops each source's
      periodic measurement; wakeup() powers the hardware back up but does not restart
      measurement -- restart it explicitly on the units you want sampling again.
     */
    bool sleep();

    /*!
      @brief Bring every added source back from sleep()
      @return True if at least one source actually woke
      @details Calls IMUDataSource::wakeup() on all sources and then discards the internal
      clock state, so the first update() after waking re-seeds its timebase instead of handing
      the filter one enormous dt spanning the whole sleep. Measurement is not restarted: call
      startPeriodicMeasurement() on the units that should resume sampling.
     */
    bool wakeup();

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

    /*!
      @brief Zero the publicly reported yaw at the current heading
      @details Samples the active filter's current fused yaw (the same value
      quaternion()/euler() would derive from it) and stores a world-frame Z-axis
      rotation of the opposite angle as an internal tare offset. From then on every
      quaternion()/euler() call composes this offset on top of the filter's raw
      attitude, so the reported yaw reads ~0 right after this call while roll and
      pitch are unaffected. The filter's own internal attitude estimate -- and, on a
      9-axis setup, its alignment to magnetic north -- is left untouched; only the
      publicly reported attitude is shifted. Calling this again re-zeros to whatever
      heading the filter has reached by then. See clearYawTare() to remove the offset.
      No-op (tare stays whatever it already was) if no filter has been selected yet.
     */
    void tareYaw();

    /*!
      @brief Remove a previously applied tareYaw() offset
      @details Resets the internal yaw tare to identity, so quaternion()/euler() once
      again report the filter's raw heading as-is (e.g. relative to magnetic north on
      a 9-axis setup).
     */
    void clearYawTare();

    //! @brief Most recently fused physical-unit sample
    inline const imu_physical_data_t& latest() const
    {
        return _latest;
    }

    //! @brief Whether an accelerometer source has been registered (see add())
    inline bool hasAccel() const
    {
        return _accel_src != nullptr;
    }

    //! @brief Whether a gyroscope source has been registered (see add())
    inline bool hasGyro() const
    {
        return _gyro_src != nullptr;
    }

    //! @brief Whether a temperature source has been registered (see add())
    inline bool hasTemp() const
    {
        return _temp_src != nullptr;
    }

    /*!
      @brief Whether at least one valid magnetometer sample has been seen
      @details False on a 6-axis setup, or before the first non-overflow mag sample arrives
      (see the "Magnetometer overflow" note on this class). latest().mag stays at its last
      valid value (or zero) while this reads false.
     */
    inline bool hasValidMag() const
    {
        return _has_valid_mag;
    }

    /*!
      @brief Microsecond timestamp of the most recent self-clocked update()
      @details Stamped only by the update() overload that measures dt itself; update(const float
      dt) leaves it untouched, so this stays 0 for callers driving the controller entirely from
      their own timebase.
     */
    inline unsigned long lastMicros() const
    {
        return _last_micros;
    }

private:
    // Per-capability auto-calibration state (M5Unified IMU_Class::offset_point_t, ported to
    // physical-unit floats -- see ahrs_controller.cpp for update_stillness()/calibrate()).
    struct axis_offset_state_t {
        float offset[3]{};           // Current offset estimate, subtracted from every sample
        float prev[3]{};             // Previous physical sample, for stillness diffing
        float avg[3]{};              // Exponential moving average of recent samples
        float radius{};              // Expected magnitude of (sample - offset) at rest
        float tolerance{};           // Tolerance band around radius before nudging the offset
        float noise{};               // Per-sample movement below which stillness accumulates
        uint8_t average_shifter{1};  // avg update rate: avg += diff / (1 << average_shifter)
        uint8_t stillness{};         // 0 = moving, saturates toward 255 while stationary
        uint8_t strength{};          // 0 = auto-calibration off, 1-255 = adjustment speed
        // Radius adaptation: 0 keeps `radius` fixed (correct for accel, whose sphere is 1G
        // everywhere, and for gyro, whose sphere is the origin). Non-zero makes `radius` track the
        // observed magnitude at rate 1/(1 << radius_shift) per calibrate() call -- needed for the
        // magnetometer, whose sphere is the local field strength and therefore varies by more than
        // a factor of two across the planet (~25-65uT). Must stay much slower than the offset
        // correction itself, so that it settles on the average over orientations rather than
        // chasing each individual sample (which would cancel the correction entirely).
        uint8_t radius_shift{};
        bool radius_seeded{};  // First observed magnitude has been taken as the initial radius

        uint8_t update_stillness(const float sample[3]);
        void calibrate();
    };

    static constexpr size_t MAX_SOURCES{4};

    static bool is_mag_overflow(const int32_t mag[3]);
    static void apply_offset_and_calibrate(axis_offset_state_t& cal, float v[3]);

    bool find_source_index(const IMUDataSource& src, size_t& out_index) const;
    Capability drain_and_fuse(const float dt);
    void begin_gyro_collect();
    bool end_gyro_collect();
    void recompute_composed(const size_t index);  // Fold _user_axis_order onto one source's fixed order
    void recompute_all_composed();                // ...for every added source

    IMUDataSource* _sources[MAX_SOURCES]{};
    AxisOrder _axis_orders[MAX_SOURCES]{};      // Fixed board correction for Accel/Gyro/Temp
    AxisOrder _mag_axis_orders[MAX_SOURCES]{};  // Fixed board correction for Mag (writeMagAxisOrder())
    AxisOrder _user_axis_order{};               // Global user remap (writeUserAxisOrder())

    // Fixed order folded with the user order; these are what update() actually applies.
    AxisOrder _composed_axis_orders[MAX_SOURCES]{};
    AxisOrder _composed_mag_axis_orders[MAX_SOURCES]{};
    size_t _source_count{};

    IMUDataSource* _accel_src{};
    IMUDataSource* _gyro_src{};
    IMUDataSource* _mag_src{};
    IMUDataSource* _temp_src{};

    AhrsFilterBase* _owned_filter{};  // Non-null only when constructed by selectFilter()
    AhrsFilterBase* _filter{};        // Active filter (owned or injected); nullptr until chosen

    // World-frame Z rotation composed onto the filter's raw quaternion by quaternion()/euler()
    // (see tareYaw()/clearYawTare()); identity (no-op) by default.
    m5::math::Quaternion _yaw_tare{};

    imu_physical_data_t _latest{};
    bool _has_valid_mag{false};

    unsigned long _last_micros{};
    bool _has_last_micros{false};
    float _dt_ema{};
    bool _has_dt_ema{false};
    // Time from update() calls that drained no gyro sample, not yet handed to the filter
    float _pending_dt{};

    axis_offset_state_t _accel_cal{};
    axis_offset_state_t _gyro_cal{};
    axis_offset_state_t _mag_cal{};

    bool _gyro_collecting{false};
    float _gyro_collect_sum[3]{};
    uint32_t _gyro_collect_count{};
};

}  // namespace imu
}  // namespace m5
#endif
