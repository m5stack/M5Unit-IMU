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

bool MotionEventController::add(MotionEventSource& src)
{
    if (_count >= MAX_SOURCES) return false;

    const auto spec = src.motionEventSpec();
    auto claimed    = MotionEventKind::None;
    for (uint32_t bit = 1; bit != 0; bit <<= 1) {
        const auto k = static_cast<MotionEventKind>(bit);
        if (any(spec & k) && find_owner(k) == nullptr) {
            claimed |= k;
        }
    }
    _entries[_count].src   = &src;
    _entries[_count].claim = claimed;
    ++_count;
    return true;
}

MotionEventKind MotionEventController::supported() const
{
    auto acc = MotionEventKind::None;
    for (uint8_t i = 0; i < _count; ++i) {
        acc |= _entries[i].src->motionEventSpec();
    }
    return acc;
}

MotionEventKind MotionEventController::enabledEvents() const
{
    auto acc = MotionEventKind::None;
    for (uint8_t i = 0; i < _count; ++i) {
        acc |= _entries[i].src->enabledEvents();
    }
    return acc;
}

bool MotionEventController::has(MotionEventKind kind) const
{
    return any(supported() & kind);
}

MotionEventController::Entry* MotionEventController::find_owner(MotionEventKind kind)
{
    for (uint8_t i = 0; i < _count; ++i) {
        if (any(_entries[i].claim & kind)) return &_entries[i];
    }
    return nullptr;
}

const MotionEventController::Entry* MotionEventController::find_owner(MotionEventKind kind) const
{
    for (uint8_t i = 0; i < _count; ++i) {
        if (any(_entries[i].claim & kind)) return &_entries[i];
    }
    return nullptr;
}

bool MotionEventController::enable(MotionEventKind kind, const void* cfg)
{
    auto* e = find_owner(kind);
    return e && e->src->enableEvent(kind, cfg);
}

bool MotionEventController::disable(MotionEventKind kind)
{
    auto* e = find_owner(kind);
    return e && e->src->disableEvent(kind);
}

bool MotionEventController::disableAll()
{
    bool ok = true;
    for (uint8_t i = 0; i < _count; ++i) {
        if (!_entries[i].src->disableAllEvents()) ok = false;
    }
    return ok;
}

MotionEventKind MotionEventController::update()
{
    return update(0.020f);
}

MotionEventKind MotionEventController::update(const float dt)
{
    (void)dt;  // HW sources poll their own status; dt is consumed by SW plugins fed from the sketch.
    _latest = MotionEventStatus{};
    for (uint8_t i = 0; i < _count; ++i) {
        MotionEventStatus s{};
        if (_entries[i].src->readEventStatus(s)) {
            _latest.fired |= s.fired;
            if (s.step_count) _latest.step_count = s.step_count;
            if (s.gesture_type != GestureType::None) _latest.gesture_type = s.gesture_type;
            if (s.activity_type != ActivityType::Unknown) _latest.activity_type = s.activity_type;
        }
    }
    return _latest.fired;
}

bool MotionEventController::latest(MotionEventStatus& out) const
{
    out = _latest;
    return any(_latest.fired);
}

}  // namespace imu
}  // namespace m5
