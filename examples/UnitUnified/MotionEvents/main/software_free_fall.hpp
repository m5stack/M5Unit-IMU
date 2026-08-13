/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file software_free_fall.hpp
  @brief Software motion event plugin sample used by the MotionEvents example.
  @note This header lives inside the example folder on purpose: software-based motion
  detection is a user extension point, not part of the library. Users who need free-fall
  detection for their own sketches can copy this file as a template.
 */
#ifndef EXAMPLE_SOFTWARE_FREE_FALL_HPP
#define EXAMPLE_SOFTWARE_FREE_FALL_HPP

#include <cmath>
#include "imu/motion_events.hpp"

namespace example {

//! @brief User-defined FreeFall bit.
//! @note Convention: library HW events occupy bits from bit 0 upward
//! (currently 0-6); user extensions occupy bits from bit 31 downward.
//! Starting user bits at the top keeps them collision-free even as new HW
//! events land in future library releases.
static constexpr auto FreeFallBit = static_cast<m5::imu::MotionEventKind>(1u << 31);

/*!
  @class SoftwareFreeFall
  @brief Fires FreeFall when |a| < threshold_g holds for at least duration_ms.
 */
class SoftwareFreeFall : public m5::imu::SoftwareMotionEventBase {
public:
    struct Config {
        float threshold_g;     //!< @brief Acceleration magnitude threshold (G)
        uint16_t duration_ms;  //!< @brief Consecutive time below threshold to fire

        Config() : threshold_g(0.3f), duration_ms(100)
        {
        }
        Config(float g, uint16_t ms) : threshold_g(g), duration_ms(ms)
        {
        }
    };

    explicit SoftwareFreeFall(const Config cfg = Config{}) : _cfg{cfg}
    {
    }

    m5::imu::MotionEventKind motionEventSpec() const override
    {
        return FreeFallBit;
    }

    m5::imu::MotionEventKind enabledEvents() const override
    {
        return _enabled ? FreeFallBit : m5::imu::MotionEventKind::None;
    }

    bool enableEvent(m5::imu::MotionEventKind kind, const void* cfg) override
    {
        if (kind != FreeFallBit) return false;
        if (cfg) _cfg = *static_cast<const Config*>(cfg);
        _enabled  = true;
        _still_ms = 0.0f;
        _fired    = false;
        return true;
    }

    bool disableEvent(m5::imu::MotionEventKind kind) override
    {
        if (kind != FreeFallBit) return false;
        _enabled = false;
        return true;
    }

    bool readEventStatus(m5::imu::MotionEventStatus& out) override
    {
        if (_fired) {
            out.fired |= FreeFallBit;
            _fired = false;
            return true;
        }
        return false;
    }

    void update(const float dt, const m5::imu::imu_physical_data_t& s) override
    {
        if (!_enabled) return;
        const float mag = std::sqrt(s.accel[0] * s.accel[0] + s.accel[1] * s.accel[1] + s.accel[2] * s.accel[2]);
        if (mag < _cfg.threshold_g) {
            _still_ms += dt * 1000.0f;
            if (_still_ms >= static_cast<float>(_cfg.duration_ms)) {
                _fired    = true;
                _still_ms = 0.0f;
            }
        } else {
            _still_ms = 0.0f;
        }
    }

private:
    Config _cfg;
    float _still_ms{0.0f};
    bool _enabled{false};
    bool _fired{false};
};

}  // namespace example

#endif  // EXAMPLE_SOFTWARE_FREE_FALL_HPP
