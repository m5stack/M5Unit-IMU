/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file motion_events.hpp
  @brief HW-backed motion event framework (poll only)
*/
#ifndef M5_UNIT_IMU_IMU_MOTION_EVENTS_HPP
#define M5_UNIT_IMU_IMU_MOTION_EVENTS_HPP

#include <cstdint>
#include <m5_utility/stl/extension.hpp>
#include "imu_types.hpp"

namespace m5 {
namespace imu {

/*!
  @enum MotionEventKind
  @brief Motion event categories (bitwise combinable). Sources declare which they provide via motionEventSpec().
 */
enum class MotionEventKind : uint32_t {
    None              = 0,
    AnyMotion         = 1u << 0,  //!< @brief BMI270 any_motion / MPU-6886 WoM
    NoMotion          = 1u << 1,  //!< @brief BMI270 no_motion
    SignificantMotion = 1u << 2,  //!< @brief BMI270 significant_motion
    StepCounter       = 1u << 3,  //!< @brief BMI270 step_counter / detector
    ActivityChange    = 1u << 4,  //!< @brief BMI270 activity_change_recognition; type via ActivityType
    PickUpGesture     = 1u << 5,  //!< @brief Raise-to-view / wake gesture (Android WAKE_GESTURE equivalent); BMI270 wrist_wear_wakeup
    TapGesture        = 1u << 6,  //!< @brief Single tap detection; reserved, no BMI270 support at this tier
    DoubleTapGesture  = 1u << 7,  //!< @brief Double tap detection; reserved, no BMI270 support at this tier
    TiltGesture       = 1u << 8,  //!< @brief Tilt gesture; reserved, no BMI270 support at this tier
    WristGesture      = 1u << 9,  //!< @brief BMI270 wrist_navigation_gesture; wrist-family type via GestureType
    // Reserved bit 10+ for future tiers
};

//! @brief Bitwise OR
constexpr MotionEventKind operator|(const MotionEventKind a, const MotionEventKind b)
{
    return static_cast<MotionEventKind>(m5::stl::to_underlying(a) | m5::stl::to_underlying(b));
}
//! @brief Bitwise AND
constexpr MotionEventKind operator&(const MotionEventKind a, const MotionEventKind b)
{
    return static_cast<MotionEventKind>(m5::stl::to_underlying(a) & m5::stl::to_underlying(b));
}
//! @brief Bitwise NOT
constexpr MotionEventKind operator~(const MotionEventKind a)
{
    return static_cast<MotionEventKind>(~m5::stl::to_underlying(a));
}
//! @brief Bitwise OR-assign
inline MotionEventKind& operator|=(MotionEventKind& a, const MotionEventKind b)
{
    a = a | b;
    return a;
}
//! @brief Bitwise AND-assign
inline MotionEventKind& operator&=(MotionEventKind& a, const MotionEventKind b)
{
    a = a & b;
    return a;
}
//! @brief Any bit set?
constexpr bool any(const MotionEventKind c)
{
    return c != MotionEventKind::None;
}

/*!
  @brief Activity classification reported when ActivityChange event fires
  @note Chip-specific behavior:
   - BMI270 supports: Unknown, Stationary, Walking, Running
   - Other chips (LSM6DSOX/ICM/BHI) may report additional types (Cycling/InVehicle/etc.)
   - Query MotionEventSource::supportedActivityTypes() for the chip-specific set
 */
enum class ActivityType : uint8_t {
    Unknown    = 0,
    Stationary = 1,
    Walking    = 2,
    Running    = 3,
    Cycling    = 4,
    InVehicle  = 5,
    Sitting    = 6,
    Standing   = 7,
    Tilting    = 8,
    // 9-30 reserved for future
    Other = 31,  //!< @brief Explicit "classified but not in this enum"
};

/*!
  @brief Gesture type reported when WristGesture event fires
  @note Chip-specific behavior:
   - BMI270 supports the five wrist-family gestures below
   - Values 6-31 reserved for future wrist-designed chips
   - Non-wrist gestures (Tap/DoubleTap/PickUp) surface as their own MotionEventKind bits
 */
enum class GestureType : uint8_t {
    None             = 0,
    WristPushArmDown = 1,
    WristPivotUp     = 2,
    WristShake       = 3,
    WristFlickIn     = 4,
    WristFlickOut    = 5,
    // 6-31 reserved
};

/*!
  @struct MotionEventStatus
  @brief Latched event snapshot returned by readEventStatus() and Controller::latest()
 */
struct MotionEventStatus {
    MotionEventKind fired{MotionEventKind::None};       //!< @brief Bitmap of events fired since last read
    uint32_t step_count{0};                             //!< @brief Set when StepCounter fired (BMI270)
    ActivityType activity_type{ActivityType::Unknown};  //!< @brief Latest activity classification (updated when ActivityChange fires)
    GestureType gesture_type{GestureType::None};        //!< @brief Latest wrist gesture (updated when WristGesture fires)
};

/*!
  @struct AnyMotionConfig
  @brief Configuration for MotionEventKind::AnyMotion (BMI270 any_motion / MPU-6886 WoM)
  @note Chip-specific behavior:
   - BMI270: uses feature engine ANYMO_1/ANYMO_2 (§4.8.2), 11-bit threshold at ~0.48mg/LSB scale
   - MPU-6886: uses per-axis Wake-on-Motion registers ACCEL_WOM_X/Y/Z_THR (0x20-0x22) at 4mg/LSB;
     duration_ms is ignored (no consecutive-sample HW gate on this chip)
 */
struct AnyMotionConfig {
    uint16_t threshold_mg{500};  //!< @brief Motion threshold in mg (driver converts to LSB)
    uint16_t duration_ms{40};    //!< @brief Consecutive time above threshold (BMI270 only; MPU-6886 ignores)
    bool axis_x{true};           //!< @brief Include X axis in detection
    bool axis_y{true};           //!< @brief Include Y axis
    bool axis_z{true};           //!< @brief Include Z axis
};

/*!
  @struct NoMotionConfig
  @brief BMI270 no_motion feature configuration (§4.8.3)
  @note Chip-specific behavior:
   - BMI270: uses feature engine NOMO_1/NOMO_2 (§4.8.3), 13-bit duration @50Hz, 11-bit threshold
     at ~0.48mg/LSB scale
   - Other chips: not supported at this tier
 */
struct NoMotionConfig {
    uint16_t threshold_mg{70};   //!< @brief Slope threshold (default 70mg per datasheet)
    uint32_t duration_ms{5000};  //!< @brief Consecutive still time to fire (13-bit @ 50Hz)
    bool axis_x{true};           //!< @brief Include X axis in detection
    bool axis_y{true};           //!< @brief Include Y axis
    bool axis_z{true};           //!< @brief Include Z axis
    NoMotionConfig() = default;
    NoMotionConfig(uint16_t thr_mg, uint32_t dur_ms, bool x, bool y, bool z)
        : threshold_mg{thr_mg}, duration_ms{dur_ms}, axis_x{x}, axis_y{y}, axis_z{z}
    {
    }
};

/*!
  @struct SignificantMotionConfig
  @brief BMI270 significant_motion feature configuration (§4.8.4, Android SIGNIFICANT_MOTION)
  @note Chip-specific behavior:
   - BMI270: uses feature engine SIGMO_1 / SIGMO_2 (§4.8.4), 16-bit block_size in 50Hz samples
     (duration_ms is converted internally). Once triggered the chip auto-disables the feature;
     re-enable is required to arm the next detection (Android SIGNIFICANT_MOTION convention).
   - Other chips: not supported at this tier
 */
struct SignificantMotionConfig {
    uint32_t duration_ms{5000};  //!< @brief block_size in ms (16-bit @ 50Hz, default 5 sec)
    SignificantMotionConfig() = default;
    explicit SignificantMotionConfig(uint32_t dur_ms) : duration_ms{dur_ms}
    {
    }
};

/*!
  @struct StepCounterConfig
  @brief BMI270 step counter/detector configuration (§4.8.8)
  @note Chip-specific behavior:
   - BMI270: watermark_level (SC_26 bits 9:0) implicitly has a 20x scaling factor; the driver
     rounds watermark_steps to the nearest 20 and logs an M5_LIB_LOGW when the requested value
     cannot be represented exactly. See stepCounterResolutionSteps() for the query API.
   - Other chips: may resolve at 1-step granularity (LSM6DSOX FSM) or firmware-defined (ICM DMP)
 */
struct StepCounterConfig {
    //! @brief Firing mode
    enum class Mode : uint8_t {
        Counter,   //!< @brief Fire when cumulative step count reaches watermark_steps
        Detector,  //!< @brief Fire on every single step (low-latency)
    };
    Mode mode{Mode::Counter};
    uint16_t watermark_steps{1};  //!< @brief Watermark in raw steps for Counter mode (ignored in Detector).
                                  //!<        Chip-specific hardware granularity may round this value;
                                  //!<        query MotionEventSource::stepCounterResolutionSteps() to check.
    StepCounterConfig() = default;
    StepCounterConfig(Mode m, uint16_t wm) : mode{m}, watermark_steps{wm}
    {
    }
};

/*!
  @struct ActivityChangeConfig
  @brief Activity classifier event configuration
  @note Chip-specific behavior:
   - BMI270: enables SC_26.en_activity; activity type is auto-detected, no threshold params
   - Other chips: may expose classification sensitivity in future extensions
 */
struct ActivityChangeConfig {
    ActivityChangeConfig() = default;
};

/*!
  @struct PickUpGestureConfig
  @brief Raise-to-view / wake gesture configuration (Android WAKE_GESTURE equivalent)
  @note Chip-specific behavior:
   - BMI270: enables the wrist wear wakeup feature (WR_WAKEUP_1); angle/tilt thresholds use
     datasheet defaults from §4.8.6 Table 14. No user-tunable params at this tier.
   - Other chips (LSM/ICM/BHI): would implement raise-to-wake via their own algorithms
 */
struct PickUpGestureConfig {
    PickUpGestureConfig() = default;
};

/*!
  @struct WristGestureConfig
  @brief Wrist-family gesture detector configuration
  @note Chip-specific behavior:
   - BMI270: enables WR_GEST_1..4 (§4.8.7); wearable_arm defaults to Left (chip default 0),
     other timing/threshold params use datasheet defaults. Not exposed at this tier;
     may be extended in a future release.
   - Wrist-specific by definition; non-wrist chips report supportedGestureTypes()==0 and
     enableEvent(WristGesture, ...) returns false.
 */
struct WristGestureConfig {
    WristGestureConfig() = default;
};

/*!
  @class MotionEventSource
  @brief HW-backed motion event provider (implemented by unit drivers like UnitBMI270 / UnitMPU6886)
  @details Mirrors AHRS side IMUDataSource: capability query + enable/disable/status read the 4 API to provide
 */
class MotionEventSource {
public:
    virtual ~MotionEventSource() = default;

    //! @brief This source provides HW event bitmap (constant, callable before begin/enable)
    virtual MotionEventKind motionEventSpec() const = 0;

    //! @brief Currently enabled event bitmap (subset of spec)
    virtual MotionEventKind enabledEvents() const = 0;

    //! @brief Enable specified event
    //! @param kind Event type to enable (single bit)
    //! @param cfg Pointer to corresponding config struct (void* because type differs per kind)
    //! @return true success, false chip unsupported / ODR conflict / register write failed
    virtual bool enableEvent(MotionEventKind kind, const void* cfg) = 0;

    //! @brief Disable specified event (including restoring corresponding chip registers)
    //! @return true success
    virtual bool disableEvent(MotionEventKind kind) = 0;

    //! @brief Sequentially disable all events in spec via disableEvent
    //! @details Default impl iterates spec bits and calls disableEvent. Override if chip provides
    //! optimized bulk disable.
    virtual bool disableAllEvents();

    //! @brief Read latched flags into MotionEventStatus aggregate (read clears internal latch)
    //! @param[out] out fired flags and event-specific payload
    //! @return true if any event was fired since last read
    virtual bool readEventStatus(MotionEventStatus& out) = 0;

    /*!
      @brief Hardware granularity of the step counter fire watermark, in raw steps.
      @return 1 if the chip resolves 1-step watermarks (default); larger values for chips
              that round to a coarser boundary (e.g. BMI270 returns 20).
     */
    virtual uint16_t stepCounterResolutionSteps() const
    {
        return 1;
    }

    /*!
      @brief Bit mask of ActivityType values this chip can classify
      @return OR of (1u << static_cast<uint8_t>(ActivityType::X)) for each supported type.
              Default 0 means activity classification is not supported.
     */
    virtual uint32_t supportedActivityTypes() const
    {
        return 0;
    }

    /*!
      @brief Bit mask of GestureType values this chip can detect via the WristGesture event
      @return OR of (1u << static_cast<uint8_t>(GestureType::X)) for each supported gesture.
              Default 0 means wrist-family gestures are not supported. Non-wrist gestures
              (Tap/DoubleTap/PickUp) are queried through motionEventSpec(), not this mask.
     */
    virtual uint32_t supportedGestureTypes() const
    {
        return 0;
    }
};

/*!
  @class SoftwareMotionEventBase
  @brief User-extensible base for SW motion event plugins (for examples only, lib contains no concrete impl)
  @details MotionEventController::update() injects dt + physical sample to drive internal detection.
  Corresponds to AHRS side AhrsFilterBase for user extensibility.
 */
class SoftwareMotionEventBase : public MotionEventSource {
public:
    //! @brief Controller injects 1 tick of accel/gyro/mag physical sample to drive internal detection
    //! @param dt Elapsed seconds since previous update()
    //! @param sample IMU physical sample (accel [G] / gyro [dps] / mag [uT])
    virtual void update(const float dt, const imu_physical_data_t& sample) = 0;
};

/*!
  @class MotionEventController
  @brief Aggregator: registers multiple MotionEventSource and dispatches via first-in-wins claim
  @details Mirrors AhrsController::add() claim policy: each bit is assigned to the first source
  whose spec includes it; later sources that also claim it are ignored for that bit.
 */
class MotionEventController {
public:
    //! @brief Maximum number of sources held internally (fixed upper bound to avoid std::vector on embedded)
    static constexpr uint8_t MAX_SOURCES = 4;

    MotionEventController() = default;

    //! @brief Register a source (unclaimed bits in its spec are assigned)
    //! @return true on success, false when MAX_SOURCES reached
    bool add(MotionEventSource& src);

    //! @brief OR of every registered source's spec
    MotionEventKind supported() const;

    //! @brief OR of every registered source's enabledEvents()
    MotionEventKind enabledEvents() const;

    //! @brief Whether kind is included in supported()
    bool has(MotionEventKind kind) const;

    //! @brief Dispatch to the source claiming kind
    bool enable(MotionEventKind kind, const void* cfg);
    bool disable(MotionEventKind kind);

    //! @brief Disable every kind on every registered source
    bool disableAll();

    //! @brief Poll every source's readEventStatus() and aggregate the results
    //! @return Bitwise OR of events that newly fired during this update() call
    MotionEventKind update();
    MotionEventKind update(const float dt);

    //! @brief Snapshot of the most recent update() result
    bool latest(MotionEventStatus& out) const;

private:
    struct Entry {
        MotionEventSource* src{nullptr};
        MotionEventKind claim{MotionEventKind::None};
    };
    Entry _entries[MAX_SOURCES]{};
    uint8_t _count{0};
    MotionEventStatus _latest{};

    Entry* find_owner(MotionEventKind kind);
    const Entry* find_owner(MotionEventKind kind) const;
};

}  // namespace imu
}  // namespace m5

#endif  // M5_UNIT_IMU_IMU_MOTION_EVENTS_HPP
