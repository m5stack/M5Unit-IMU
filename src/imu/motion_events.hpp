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
    WristWakeup       = 1u << 4,  //!< @brief BMI270 wrist_wear_wakeup
    WristGesture      = 1u << 5,  //!< @brief BMI270 wrist_navigation_gesture
    ActivityChange    = 1u << 6,  //!< @brief BMI270 activity_change_recognition
};

//! @brief Bitwise OR
constexpr MotionEventKind operator|(const MotionEventKind a, const MotionEventKind b)
{
    return static_cast<MotionEventKind>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
//! @brief Bitwise AND
constexpr MotionEventKind operator&(const MotionEventKind a, const MotionEventKind b)
{
    return static_cast<MotionEventKind>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
//! @brief Bitwise NOT
constexpr MotionEventKind operator~(const MotionEventKind a)
{
    return static_cast<MotionEventKind>(~static_cast<uint32_t>(a));
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
  @struct MotionEventStatus
  @brief Latched event snapshot returned by readEventStatus() and Controller::latest()
 */
struct MotionEventStatus {
    MotionEventKind fired{MotionEventKind::None};  //!< @brief Bitmap of events fired since last read
    uint32_t step_count{0};                        //!< @brief Set when StepCounter fired (BMI270)
    uint8_t wrist_gesture{0};                      //!< @brief Set when WristGesture fired (BMI270)
    uint8_t activity_kind{0};                      //!< @brief Set when ActivityChange fired (BMI270)
};

/*!
  @struct AnyMotionConfig
  @brief Configuration for MotionEventKind::AnyMotion (BMI270 any_motion / MPU-6886 WoM)
 */
struct AnyMotionConfig {
    uint16_t threshold_mg{500};  //!< @brief Motion threshold in mg (driver converts to LSB)
    uint16_t duration_ms{40};    //!< @brief Consecutive time above threshold (BMI270 only; MPU-6886 ignores)
    bool axis_x{true};           //!< @brief Include X axis in detection
    bool axis_y{true};           //!< @brief Include Y axis
    bool axis_z{true};           //!< @brief Include Z axis
};

}  // namespace imu
}  // namespace m5

#endif  // M5_UNIT_IMU_IMU_MOTION_EVENTS_HPP
