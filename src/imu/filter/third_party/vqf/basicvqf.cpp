/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
// Not vendored (unlike basicvqf.hpp/basicvqf.cpp.inc in this directory): this is the
// compilation unit that actually builds upstream's basicvqf.cpp, renamed byte-for-byte to
// basicvqf.cpp.inc here so it is not picked up a second time by source-file globbing (both
// PlatformIO's `build_src_filter` and the Arduino IDE/Arduino-CLI compile every *.cpp under
// src/ they find).
//
// VQF_SINGLE_PRECISION must be defined here, before including basicvqf.cpp.inc, so that
// upstream's `vqf_real_t` (see basicvqf.hpp) resolves to `float` for the BasicVQF member
// functions defined in that file -- matching this repository's float-based AhrsFilterBase
// (see ../vqf.hpp) instead of upstream's default `double`. Defining the macro only at the
// adapter's include site (../vqf.hpp/../vqf.cpp) would not be enough: basicvqf.cpp.inc's own
// translation unit needs the very same definition of `vqf_real_t`, or the BasicVQF layout it
// was compiled with (arrays of double) would silently disagree with the layout the adapter
// links against (arrays of float) -- an ODR/ABI mismatch, not merely a numeric-precision one.
#define VQF_SINGLE_PRECISION
#include "basicvqf.cpp.inc"
