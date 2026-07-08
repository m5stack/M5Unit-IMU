/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file bmm150_oracle.hpp
 *
 * Test-only oracle ported from Bosch Sensortec's official BMM150 reference
 * driver (bmm150.cpp / bmm150_defs.h, as vendored under
 * M5StickC-Plus/examples/Hat/ENVII_SHT30_BMP280/). The device I/O has been
 * stripped so only the pure trim-compensation math (compensate_x/y/z)
 * remains, taking the trim registers and raw ADC readings as plain
 * arguments.
 *
 * This file exists solely to cross-check the independent reimplementation in
 * src/unit/unit_bmm150_data.hpp against Bosch's own arithmetic; it is not
 * shipped with the library (test/ only) so the BSD-3-Clause license here
 * does not mix with the MIT-licensed src/ tree.
 */
#ifndef M5_UNIT_IMU_TEST_BMM150_ORACLE_HPP
#define M5_UNIT_IMU_TEST_BMM150_ORACLE_HPP

#include <cstdint>

namespace oracle {

constexpr int16_t OVERFLOW_OUTPUT             = -32768;
constexpr int16_t XYAXES_FLIP_OVERFLOW_ADCVAL = -4096;
constexpr int16_t ZAXIS_HALL_OVERFLOW_ADCVAL  = -16384;
constexpr int32_t NEGATIVE_SATURATION_Z       = -32767;
constexpr int32_t POSITIVE_SATURATION_Z       = 32767;

/*!
 * @brief bmm150 trim data structure (verbatim field layout from
 * bmm150_defs.h's bmm150_trim_registers)
 */
struct trim_registers {
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
};

/*
 * @brief This internal API is used to obtain the compensated
 * magnetometer X axis data(micro-tesla) in int16_t.
 */
inline int16_t compensate_x(int16_t mag_data_x, uint16_t data_rhall, const trim_registers& trim_data)
{
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;

    /* Overflow condition check */
    if (mag_data_x != XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_x0 = data_rhall;
        } else if (trim_data.dig_xyz1 != 0) {
            process_comp_x0 = trim_data.dig_xyz1;
        } else {
            process_comp_x0 = 0;
        }
        if (process_comp_x0 != 0) {
            /* Processing compensation equations*/
            process_comp_x1  = ((int32_t)trim_data.dig_xyz1) * 16384;
            process_comp_x2  = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval           = ((int16_t)process_comp_x2);
            process_comp_x3  = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4  = (((int32_t)trim_data.dig_xy2) * (process_comp_x3 / 128));
            process_comp_x5  = (int32_t)(((int16_t)trim_data.dig_xy1) * 128);
            process_comp_x6  = ((int32_t)retval) * process_comp_x5;
            process_comp_x7  = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8  = ((int32_t)(((int16_t)trim_data.dig_x2) + ((int16_t)0xA0)));
            process_comp_x9  = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval           = ((int16_t)(process_comp_x10 / 8192));
            retval           = (retval + (((int16_t)trim_data.dig_x1) * 8)) / 16;
        } else {
            retval = OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition */
        retval = OVERFLOW_OUTPUT;
    }

    return retval;
}

/*
 * @brief This internal API is used to obtain the compensated
 * magnetometer Y axis data(micro-tesla) in int16_t.
 */
inline int16_t compensate_y(int16_t mag_data_y, uint16_t data_rhall, const trim_registers& trim_data)
{
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;

    /* Overflow condition check */
    if (mag_data_y != XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_y0 = data_rhall;
        } else if (trim_data.dig_xyz1 != 0) {
            process_comp_y0 = trim_data.dig_xyz1;
        } else {
            process_comp_y0 = 0;
        }
        if (process_comp_y0 != 0) {
            /*Processing compensation equations*/
            process_comp_y1 = (((int32_t)trim_data.dig_xyz1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval          = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t)retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)trim_data.dig_xy2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)trim_data.dig_xy1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = ((int32_t)(((int16_t)trim_data.dig_y2) + ((int16_t)0xA0)));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval          = (int16_t)(process_comp_y9 / 8192);
            retval          = (retval + (((int16_t)trim_data.dig_y1) * 8)) / 16;
        } else {
            retval = OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition*/
        retval = OVERFLOW_OUTPUT;
    }

    return retval;
}

/*
 * @brief This internal API is used to obtain the compensated
 * magnetometer Z axis data(micro-tesla) in int16_t.
 */
inline int16_t compensate_z(int16_t mag_data_z, uint16_t data_rhall, const trim_registers& trim_data)
{
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;

    if (mag_data_z != ZAXIS_HALL_OVERFLOW_ADCVAL) {
        if ((trim_data.dig_z2 != 0) && (trim_data.dig_z1 != 0) && (data_rhall != 0) && (trim_data.dig_xyz1 != 0)) {
            /*Processing compensation equations*/
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t)trim_data.dig_xyz1);
            process_comp_z1 = (((int32_t)trim_data.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - trim_data.dig_z4)) * 32768);
            process_comp_z3 = ((int32_t)trim_data.dig_z1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval          = ((process_comp_z2 - process_comp_z1) / (trim_data.dig_z2 + process_comp_z4));

            /* saturate result to +/- 2 micro-tesla */
            if (retval > POSITIVE_SATURATION_Z) {
                retval = POSITIVE_SATURATION_Z;
            } else {
                if (retval < NEGATIVE_SATURATION_Z) retval = NEGATIVE_SATURATION_Z;
            }
            /* Conversion of LSB to micro-tesla*/
            retval = retval / 16;
        } else {
            retval = OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition*/
        retval = OVERFLOW_OUTPUT;
    }

    return (int16_t)retval;
}

}  // namespace oracle

#endif
