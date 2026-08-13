/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "motion_events.hpp"

namespace m5 {
namespace imu {

bool MotionEventSource::disableAllEvents()
{
    const auto spec = motionEventSpec();
    bool ok         = true;
    for (uint32_t bit = 1; bit != 0; bit <<= 1) {
        const auto k = static_cast<MotionEventKind>(bit);
        if (any(spec & k)) {
            if (!disableEvent(k)) ok = false;
        }
    }
    return ok;
}

}  // namespace imu
}  // namespace m5
