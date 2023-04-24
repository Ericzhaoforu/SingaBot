/****************************************************************************************
@file:AP_BattMonitor_INA2xx.h
@brief:this library is based on INA series power monitor sensors (INA226,INA260 and INA219).
       All three of the sensor's are I2C based.
*****************************************************************************************
*/

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_BattMonitor_Backend.h"
#include <AP_Param/AP_Param.h>
#include <utility>

#if AP_BATTERY_INA2XX_ENABLED

//enable or disable only one of the sensor at a given time (enable at least one of the following sensor)
#define HAL_BATTMON_INA260_ENABLED  0                 
#define HAL_BATTMON_INA219_ENABLED  1
#define HAL_BATTMON_INA226_ENABLED  0


class AP_BattMonitor_INA2XX : public AP_BattMonitor_Backend
{
public:
    /// Constructor
    AP_BattMonitor_INA2XX(AP_BattMonitor &mon,
                          AP_BattMonitor::BattMonitor_State &mon_state,
                          AP_BattMonitor_Params &params);

    bool has_cell_voltages() const override { return false; }
    bool has_temperature() const override { return false; }
    bool has_current() const override { return true; }
    bool reset_remaining(float percentage) override { return false; }
    bool get_cycle_count(uint16_t &cycles) const override { return false; }
    bool has_consumed_energy() const override {return false;}

    void init(void) override;
    void read() override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    void configure(void);
    bool read_word_signed(const uint8_t reg, int16_t& data) const;
    bool read_word_unsigned(const uint8_t reg, uint16_t& data) const;
    bool write_word(const uint8_t reg, const uint16_t data) const;
    void timer(void);

    AP_Int8 _i2c_bus;
    AP_Int8 _i2c_address;
    bool _configured;
    bool _device_detected = false;
    bool _callback_registered;
    uint32_t _failed_reads;
    uint32_t _last_configure_ms;

    struct {
        uint16_t _count;
        float _volt_sum;
        float _current_sum;
        HAL_Semaphore sem;
    } accumulate;
    
#if HAL_BATTMON_INA260_ENABLED
    /*Current and Voltage LSB variables for INA260*/
    static constexpr float _current_LSB = 0.00125;   //1.25mA
    static constexpr float _voltage_LSB = 0.00125;   //1.25mV
#endif

#if HAL_BATTMON_INA219_ENABLED
    /*Current and Voltage LSB variables for INA219*/
    float _current_LSB;
    static constexpr float _voltage_LSB = 0.004;     //4mV for bus voltage
#endif

#if HAL_BATTMON_INA226_ENABLED
    /*Current and Voltage LSB variables for INA226*/
    float _current_LSB;
    static constexpr float _voltage_LSB = 0.00125;     //1.25mV for bus voltage
#endif
};

#endif // AP_BATTERY_INA2XX_ENABLED
