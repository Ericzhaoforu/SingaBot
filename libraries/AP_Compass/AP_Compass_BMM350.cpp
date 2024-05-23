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
#include "AP_Compass_BMM350.h"

#if AP_COMPASS_BMM350_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define BMM350_REG_CHIP_ID              0x00
#define BMM350_REG_PMU_CMD_AGGR_SET     0x04
#define BMM350_REG_PMU_CMD_AXIS_EN      0x05
#define BMM350_REG_PMU_CMD              0x06
#define BMM350_REG_PMU_CMD_STATUS_0     0x07
#define BMM350_REG_INT_CTRL             0x2E
#define BMM350_REG_MAG_X_XLSB           0x31
#define BMM350_REG_OTP_CMD              0x50
#define BMM350_REG_OTP_DATA_MSB         0x52
#define BMM350_REG_OTP_DATA_LSB         0x53
#define BMM350_REG_OTP_STATUS           0x55
#define BMM350_REG_CMD                  0x7E

// OTP(one-time programmable memory)
#define BMM350_OTP_CMD_DIR_READ         (0x01<<5U)
#define BMM350_OTP_CMD_PWR_OFF_OTP      (0x04<<5U)

#define BMM350_OTP_STATUS_ERROR_MASK    0xE0
#define BMM350_OTP_STATUS_CMD_DONE      0x01

#define BMM350_TEMP_OFF_SENS            0x0D
#define BMM350_MAG_OFFSET_X             0x0E
#define BMM350_MAG_OFFSET_Y             0x0F
#define BMM350_MAG_OFFSET_Z             0x10
#define BMM350_MAG_SENS_X               0x10
#define BMM350_MAG_SENS_Y               0x11
#define BMM350_MAG_SENS_Z               0x11
#define BMM350_MAG_TCO_X                0x12
#define BMM350_MAG_TCO_Y                0x13
#define BMM350_MAG_TCO_Z                0x14
#define BMM350_MAG_TCS_X                0x12
#define BMM350_MAG_TCS_Y                0x13
#define BMM350_MAG_TCS_Z                0x14
#define BMM350_MAG_DUT_T_0              0x18
#define BMM350_CROSS_X_Y                0x15
#define BMM350_CROSS_Y_X                0x15
#define BMM350_CROSS_Z_X                0x16
#define BMM350_CROSS_Z_Y                0x16
#define BMM350_SENS_CORR_Y              0.01f
#define BMM350_TCS_CORR_Z               0.0001f

#define BMM350_CMD_SOFTRESET            0xB6
#define BMM350_INT_MODE_PULSED          (0<<0U)
#define BMM350_INT_POL_ACTIVE_HIGH      (1<<1U)
#define BMM350_INT_OD_PUSHPULL          (1<<2U)
#define BMM350_INT_OUTPUT_DISABLE       (0<<3U)
#define BMM350_INT_DRDY_EN              (1<<7U)

// Averaging performance
#define BMM350_AVERAGING_4              (0x02 << 4U)
#define BMM350_AVERAGING_8              (0x03 << 4U)

// Output data rate
#define BMM350_ODR_100HZ                0x04
#define BMM350_ODR_50HZ                 0x05

// Power modes
#define BMM350_PMU_CMD_SUSPEND_MODE     0x00
#define BMM350_PMU_CMD_NORMAL_MODE      0x01
#define BMM350_PMU_CMD_UPD_OAE          0x02
#define BMM350_PMU_CMD_FGR              0x05
#define BMM350_PMU_CMD_BR               0x07

// OTP data length
#define BMM350_OTP_DATA_LENGTH          32U

// Chip ID of BMM350
#define BMM350_CHIP_ID                  0x33

#define BMM350_SIGNED_12_BIT            12U
#define BMM350_SIGNED_16_BIT            16U
#define BMM350_SIGNED_24_BIT            24U

#define BMM350_XY_SENSITIVE             14.55f
#define BMM350_Z_SENSITIVE              9.0f
#define BMM350_TEMP_SENSITIVE           0.00204f
#define BMM350_XY_INA_GAIN              19.46f
#define BMM350_Z_INA_GAIN               31.0f
#define BMM350_ADC_GAIN                 (1.0f / 1.5f)
#define BMM350_LUT_GAIN                 0.714607238769531f
#define BMM350_POWER                    ((float)(1000000.0 / 1048576.0))

#define BMM350_XY_SCALE                 (BMM350_POWER / (BMM350_XY_SENSITIVE * BMM350_XY_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_Z_SCALE                  (BMM350_POWER / (BMM350_Z_SENSITIVE * BMM350_Z_INA_GAIN * BMM350_ADC_GAIN * BMM350_LUT_GAIN))
#define BMM350_TEMP_SCALE               (1.0 / (BMM350_TEMP_SENSITIVE * BMM350_ADC_GAIN * BMM350_LUT_GAIN * 1048576))

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_BMM350::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_BMM350 *sensor = new AP_Compass_BMM350(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_BMM350::AP_Compass_BMM350(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _force_external(force_external)
    , _rotation(rotation)
{
}

int32_t AP_Compass_BMM350::fix_sign(uint32_t val, uint8_t bit)
{
    int32_t power = 0;
    int32_t ret = (int32_t)val;

    switch (bit) {
        case BMM350_SIGNED_12_BIT:
            power = 2048; // 2^11
            break;
        case BMM350_SIGNED_16_BIT:
            power = 32768; // 2^15
            break;
        case BMM350_SIGNED_24_BIT:
            power = 8388608; // 2^23
            break;
        default:
            power = 0;
            break;
    }

    if (ret >= power) {
        ret -= (power * 2);
    }

    return ret;
}

/**
 * @brief Read out OTP(one-time programmable memory) data of sensor which is the compensation coefficients
 * @see https://github.com/boschsensortec/BMM350-SensorAPI
 */
bool AP_Compass_BMM350::read_otp_data()
{
    uint8_t data = 0;
    uint8_t msb = 0;
    uint8_t lsb = 0;
    uint16_t otp_data[BMM350_OTP_DATA_LENGTH];

    for (uint8_t index = 0; index < BMM350_OTP_DATA_LENGTH; index++) {
        // Set OTP address
        data = (BMM350_OTP_CMD_DIR_READ | index);
        if (!_dev->write_register(BMM350_REG_OTP_CMD, data)) {
            return false;
        }

        // Wait OTP status be ready
        do {
            hal.scheduler->delay_microseconds(300);
            // Read OTP status
            if (!read_bytes(BMM350_REG_OTP_STATUS, &data, 1) || ((data & BMM350_OTP_STATUS_ERROR_MASK) != 0)) {
                return false;
            }
        } while (!(data & BMM350_OTP_STATUS_CMD_DONE));

        // Read OTP data
        if (!read_bytes(BMM350_REG_OTP_DATA_MSB, &msb, 1) ||
            !read_bytes(BMM350_REG_OTP_DATA_LSB, &lsb, 1)) {
            return false;
        }

        otp_data[index] = ((uint16_t)(msb << 8) | lsb) & 0xFFFF;
    }

    {
        // Update magnetometer offset and sensitivity data.
        uint16_t off_x_lsb_msb, off_y_lsb_msb, off_z_lsb_msb;
        int8_t t_off;
        int8_t sens_x, sens_y, sens_z, sens_t;
        int8_t tco_x, tco_y, tco_z;
        int8_t tcs_x, tcs_y, tcs_z;
        int8_t cross_x_y, cross_y_x, cross_z_x, cross_z_y;

        off_x_lsb_msb = otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF;
        off_y_lsb_msb = ((otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) + (otp_data[BMM350_MAG_OFFSET_Y] & 0x00FF);
        off_z_lsb_msb = (otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) + (otp_data[BMM350_MAG_OFFSET_Z] & 0x00FF);
        t_off = int8_t(otp_data[BMM350_TEMP_OFF_SENS] & 0x00FF);

        // 12-bit unsigned integer to be left-aligned in a 16-bit integer
        mag_comp.dut_offset_coef.x = fix_sign(off_x_lsb_msb, BMM350_SIGNED_12_BIT);
        mag_comp.dut_offset_coef.y = fix_sign(off_y_lsb_msb, BMM350_SIGNED_12_BIT);
        mag_comp.dut_offset_coef.z = fix_sign(off_z_lsb_msb, BMM350_SIGNED_12_BIT);
        mag_comp.dut_offset_coef.temp = t_off / 5.0f;

        sens_x = (otp_data[BMM350_MAG_SENS_X] & 0xFF00) >> 8;
        sens_y = (otp_data[BMM350_MAG_SENS_Y] & 0x00FF);
        sens_z = (otp_data[BMM350_MAG_SENS_Z] & 0xFF00) >> 8;
        sens_t = (otp_data[BMM350_TEMP_OFF_SENS] & 0xFF00) >> 8;

        mag_comp.dut_sensit_coef.x = sens_x / 256.0f;
        mag_comp.dut_sensit_coef.y = (sens_y / 256.0f) + BMM350_SENS_CORR_Y;
        mag_comp.dut_sensit_coef.z = sens_z / 256.0f;
        mag_comp.dut_sensit_coef.temp = sens_t / 512.0f;

        tco_x = (otp_data[BMM350_MAG_TCO_X] & 0x00FF);
        tco_y = (otp_data[BMM350_MAG_TCO_Y] & 0x00FF);
        tco_z = (otp_data[BMM350_MAG_TCO_Z] & 0x00FF);

        mag_comp.dut_tco.x = tco_x / 32.0f;
        mag_comp.dut_tco.y = tco_y / 32.0f;
        mag_comp.dut_tco.z = tco_z / 32.0f;

        tcs_x = (otp_data[BMM350_MAG_TCS_X] & 0xFF00) >> 8;
        tcs_y = (otp_data[BMM350_MAG_TCS_Y] & 0xFF00) >> 8;
        tcs_z = (otp_data[BMM350_MAG_TCS_Z] & 0xFF00) >> 8;

        mag_comp.dut_tcs.x = tcs_x / 16384.0f;
        mag_comp.dut_tcs.y = tcs_y / 16384.0f;
        mag_comp.dut_tcs.z = (tcs_z / 16384.0f) - BMM350_TCS_CORR_Z;

        mag_comp.dut_t0 = (fix_sign(otp_data[BMM350_MAG_DUT_T_0], BMM350_SIGNED_16_BIT) / 512.0f) + 23.0f;

        cross_x_y = (otp_data[BMM350_CROSS_X_Y] & 0x00FF);
        cross_y_x = (otp_data[BMM350_CROSS_Y_X] & 0xFF00) >> 8;
        cross_z_x = (otp_data[BMM350_CROSS_Z_X] & 0x00FF);
        cross_z_y = (otp_data[BMM350_CROSS_Z_Y] & 0xFF00) >> 8;

        mag_comp.cross_axis.cross_x_y = cross_x_y / 800.0f;
        mag_comp.cross_axis.cross_y_x = cross_y_x / 800.0f;
        mag_comp.cross_axis.cross_z_x = cross_z_x / 800.0f;
        mag_comp.cross_axis.cross_z_y = cross_z_y / 800.0f;
    }

    return true;
}

/**
 * @brief Reset bit of magnetic register and wait for change to normal mode
 */
bool AP_Compass_BMM350::mag_reset_and_wait()
{
    uint8_t data = 0;

    // Get PMU command status 0 data
    if (!read_bytes(BMM350_REG_PMU_CMD_STATUS_0, &data, 1)) {
        return false;
    }

    // Check if power mode is normal
    if (data & 0x08) {
        // Set PMU command to suspend mode
        if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUSPEND_MODE)) {
            return false;
        }
        hal.scheduler->delay(6);
    }

    // Set BR(bit reset) to PMU_CMD register
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR)) {
        return false;
    }
    // Wait for sensor complete bit reset
    hal.scheduler->delay(14);

    // Get PMU command status 0 data and verify if PMU_CMD_STATUS_0 register has BR set
    if (!read_bytes(BMM350_REG_PMU_CMD_STATUS_0, &data, 1) || (((data & 0xE0) >> 5) != 0x07)) {
        return false;
    }

    // Set FGR(flux-guide reset) to PMU_CMD register
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR)) {
        return false;
    }
    hal.scheduler->delay(18);

    // Get PMU command status 0 data and verify if PMU_CMD_STATUS_0 register has FGR set
    if (!read_bytes(BMM350_REG_PMU_CMD_STATUS_0, &data, 1) || (((data & 0xE0) >> 5) != 0x05)) {
        return false;
    }

    // Switch to normal mode
    if (!set_power_mode(BMM350_POWER_MODE_NORMAL)) {
        return false;
    }

    return true;
}

/**
 * @brief Switch sensor power mode
 */
bool AP_Compass_BMM350::set_power_mode(const enum power_mode mode)
{
    uint8_t data = 0;

    // Get PMU register data as current mode
    if (!read_bytes(BMM350_REG_PMU_CMD, &data, 1)) {
        return false;
    }

    if (data == BMM350_PMU_CMD_NORMAL_MODE || data == BMM350_PMU_CMD_UPD_OAE) {
        // Set PMU command to suspend mode
        if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_POWER_MODE_SUSPEND)) {
            return false;
        }
        // wait for sensor switch to suspend mode
        hal.scheduler->delay(6);
    }

    // Set PMU command to target mode
    if (!_dev->write_register(BMM350_REG_PMU_CMD, mode)) {
        return false;
    }

    // Wait for mode change
    if (mode == BMM350_POWER_MODE_NORMAL) {
        hal.scheduler->delay(38);   // Switch from suspend mode to normal mode
    } else {
        hal.scheduler->delay(20);   // AVERAGING_4
        // hal.scheduler->delay(28);   // AVERAGING_8
    }

    return true;
}

/**
 * @brief Read bytes from sensor
 */
bool AP_Compass_BMM350::read_bytes(const uint8_t reg, uint8_t *out, const uint16_t read_len)
{
    uint8_t data[read_len + 2];

    if (!_dev->read_registers(reg, data, read_len + 2)) {
        return false;
    }

    memcpy(out, &data[2], read_len);

    return true;
}

bool AP_Compass_BMM350::init()
{
    int8_t boot_retries = 5;
    uint8_t chip_id = 0;
    uint8_t data = 0;

    _dev->get_semaphore()->take_blocking();

    // 10 retries for init
    _dev->set_retries(10);

    // Use checked registers to cope with bus errors
    _dev->setup_checked_registers(4);

    while (boot_retries--) {
        // Soft reset
        if (!_dev->write_register(BMM350_REG_CMD, BMM350_CMD_SOFTRESET)) {
            continue;
        }
        hal.scheduler->delay(24);   // Wait 24ms for soft reset complete

        // Read and verify chip ID
        if (!read_bytes(BMM350_REG_CHIP_ID, &chip_id, 1)) {
            continue;
        }
        if (chip_id == BMM350_CHIP_ID) {
            break;
        }
    }
    
    if (boot_retries == -1) {
        goto err;
    }

    // Read out OTP data
    if (!read_otp_data()) {
        goto err;
    }

    // Power off OTP
    if (!_dev->write_register(BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP)) {
        goto err;
    }

    // Magnetic reset
    if (!mag_reset_and_wait()) {
        goto err;
    }

    // Configure interrupt settings and enable DRDY
    // Set INT mode as PULSED | active polarity | PUSH_PULL | unmap | DRDY interrupt
    data = BMM350_INT_MODE_PULSED | BMM350_INT_POL_ACTIVE_HIGH | BMM350_INT_OD_PUSHPULL | BMM350_INT_OUTPUT_DISABLE | BMM350_INT_DRDY_EN;
    if (!_dev->write_register(BMM350_REG_INT_CTRL, data)) {
        goto err;
    }

    // Setup ODR and performance. 100Hz ODR and 4 average for lownoise
    if (!_dev->write_register(BMM350_REG_PMU_CMD_AGGR_SET, (BMM350_AVERAGING_4 | BMM350_ODR_100HZ))) {
        goto err;
    }

    // Update ODR and avg parameter
    if (!_dev->write_register(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE)) {
        goto err;
    }
    hal.scheduler->delay(1);    // Wait for update ODR and avg paramter

    // Enable measurement of 3 axis
    if (!_dev->write_register(BMM350_REG_PMU_CMD_AXIS_EN, 0x07)) {
        goto err;
    }

    // Switch power mode to normal mode
    if (!set_power_mode(BMM350_POWER_MODE_NORMAL)) {
        goto err;
    }

    // Lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_BMM350);
    if (!register_compass(_dev->get_bus_id(), _compass_instance)) {
        return false;
    }
    set_dev_id(_compass_instance, _dev->get_bus_id());

    // printf("BMM350: Found at address 0x%x as compass %u\n", _dev->get_bus_address(), _compass_instance);

    set_rotation(_compass_instance, _rotation);

    if (_force_external) {
        set_external(_compass_instance, true);
    }
    
    // Call timer() at 100Hz
    _dev->register_periodic_callback(1000000U/100U, FUNCTOR_BIND_MEMBER(&AP_Compass_BMM350::timer, void));

    return true;

err:
    _dev->get_semaphore()->give();
    return false;
}

void AP_Compass_BMM350::timer()
{
    struct PACKED {
        uint8_t magx[3];
        uint8_t magy[3];
        uint8_t magz[3];
        uint8_t temp[3];
    } data;

    int32_t magx_raw = 0;
    int32_t magy_raw = 0;
    int32_t magz_raw = 0;
    int32_t temp_raw = 0;

    float magx = 0.0f;
    float magy = 0.0f;
    float magz = 0.0f;
    float temp = 0.0f;

    float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;

    // Read out measurement data
    if (!read_bytes(BMM350_REG_MAG_X_XLSB, (uint8_t *)&data, sizeof(data))) {
        goto check_registers;
    }

    // Converts raw data to signed integer value
    magx_raw = fix_sign((data.magx[0] + ((uint32_t)data.magx[1] << 8) + ((uint32_t)data.magx[2] << 16)), BMM350_SIGNED_24_BIT);
    magy_raw = fix_sign((data.magy[0] + ((uint32_t)data.magy[1] << 8) + ((uint32_t)data.magy[2] << 16)), BMM350_SIGNED_24_BIT);
    magz_raw = fix_sign((data.magz[0] + ((uint32_t)data.magz[1] << 8) + ((uint32_t)data.magz[2] << 16)), BMM350_SIGNED_24_BIT);
    temp_raw = fix_sign((data.temp[0] + ((uint32_t)data.temp[1] << 8) + ((uint32_t)data.temp[2] << 16)), BMM350_SIGNED_24_BIT);

    // Convert mag lsb to uT and temp lsb to degC
    magx = (float)magx_raw * BMM350_XY_SCALE;
    magy = (float)magy_raw * BMM350_XY_SCALE;
    magz = (float)magz_raw * BMM350_Z_SCALE;
    temp = (float)temp_raw * BMM350_TEMP_SCALE;

    if (temp > 0.0f) {
        temp -= 25.49f;
    } else if (temp < 0.0f) {
        temp += 25.49f;
    }

    // Apply compensation
    temp = ((1 + mag_comp.dut_sensit_coef.temp) * temp) + mag_comp.dut_offset_coef.temp;
    // Compensate raw magnetic data
    magx = ((1 + mag_comp.dut_sensit_coef.x) * magx) + mag_comp.dut_offset_coef.x + (mag_comp.dut_tco.x * (temp - mag_comp.dut_t0));
    magx /= 1 + mag_comp.dut_tcs.x * (temp - mag_comp.dut_t0);

    magy = ((1 + mag_comp.dut_sensit_coef.y) * magy) + mag_comp.dut_offset_coef.y + (mag_comp.dut_tco.y * (temp - mag_comp.dut_t0));
    magy /= 1 + mag_comp.dut_tcs.y * (temp - mag_comp.dut_t0);

    magz = ((1 + mag_comp.dut_sensit_coef.z) * magz) + mag_comp.dut_offset_coef.z + (mag_comp.dut_tco.z * (temp - mag_comp.dut_t0));
    magz /= 1 + mag_comp.dut_tcs.z * (temp - mag_comp.dut_t0);

    cr_ax_comp_x = (magx - mag_comp.cross_axis.cross_x_y * magy) / (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y);
    cr_ax_comp_y = (magy - mag_comp.cross_axis.cross_y_x * magx) / (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y);
    cr_ax_comp_z = (magz + (magx * (mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_z_y - mag_comp.cross_axis.cross_z_x) - 
                            magy * (mag_comp.cross_axis.cross_z_y - mag_comp.cross_axis.cross_x_y * mag_comp.cross_axis.cross_z_x)) /
                    (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y));

    magx = cr_ax_comp_x;
    magy = cr_ax_comp_y;
    magz = cr_ax_comp_z;

    {
        // Apply scaler and store in field vector and convert uT to milligauss
        Vector3f field { magx * 10.0f, magy * 10.0f, magz * 10.0f };
        accumulate_sample(field, _compass_instance);
    }

check_registers:
    _dev->check_next_register();
}

void AP_Compass_BMM350::read()
{
	drain_accumulated_samples(_compass_instance);
}

#endif  // AP_COMPASS_BMM350_ENABLED
