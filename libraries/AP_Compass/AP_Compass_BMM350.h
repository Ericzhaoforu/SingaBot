/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_BMM350_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define BMM350_I2C_ADDR_MIN 0x14
#define BMM350_I2C_ADDR_MAX 0x17


class AP_Compass_BMM350 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "BMM350";

private:
    AP_Compass_BMM350(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief BMM350 magnetometer dut offset coefficient structure
     */
    struct bmm350_vector4f
    {
        float x;    // x axis
        float y;    // y axis
        float z;    // z axis
        float temp; // Temperature
    };

    /**
     * @brief BMM350 magnetometer cross axis compensation structure
     */
    struct bmm350_cross_axis
    {
        float cross_x_y;
        float cross_y_x;
        float cross_z_x;
        float cross_z_y;
    };

    /**
     * @brief bmm350 magnetometer compensate structure
     */
    struct bmm350_mag_compensate
    {
        struct bmm350_vector4f dut_offset_coef; // Structure to store dut offset coefficient
        struct bmm350_vector4f dut_sensit_coef; // Structure to store dut sensitivity coefficient
        Vector3f dut_tco;                       // Structure to store dut tco.  coefficients of the offset
        Vector3f dut_tcs;                       // Structure to store dut tcs.  coefficients of the sensitivity
        float dut_t0;                           // Initialize T0_reading parameter
        struct bmm350_cross_axis cross_axis;    // Structure to define cross axis compensation
    };

    enum power_mode
    {
        BMM350_POWER_MODE_SUSPEND     = 0,
        BMM350_POWER_MODE_NORMAL      = 1,
        BMM350_POWER_MODE_FORCED      = 3,
        BMM350_POWER_MODE_FORCED_FAST = 4
    };

    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void timer();
    int32_t fix_sign(uint32_t val, uint8_t bit);
    bool read_otp_data();
    bool mag_reset_and_wait();
    bool set_power_mode(const enum power_mode mode);
    bool read_bytes(const uint8_t reg, uint8_t *out, const uint16_t read_len);

    uint8_t _compass_instance;
    bool _force_external;
    enum Rotation _rotation;
    struct bmm350_mag_compensate mag_comp;  // Structure for mag compensate
};

#endif  // AP_COMPASS_BMM350_ENABLED
