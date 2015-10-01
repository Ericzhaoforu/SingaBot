#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux.h"
#include "Util.h"

class Linux::LinuxPWM_Sysfs {
public:
    LinuxPWM_Sysfs(uint8_t chip, uint8_t channel);
    ~LinuxPWM_Sysfs();

    enum Polarity {
        NORMAL = 0,
        INVERSE = 1,
    };

    void enable(bool enable);
    bool is_enabled();
    void set_period(uint32_t usec_period);
    uint32_t get_period();
    void set_freq(uint32_t freq);
    uint32_t get_freq();
    void set_duty_cycle(uint32_t usec_duty_cycle);
    uint32_t get_duty_cycle();
    void set_polarity(LinuxPWM_Sysfs::Polarity polarity);
    LinuxPWM_Sysfs::Polarity get_polarity();

private:
    uint8_t _chip;
    uint16_t _channel;
    int _duty_cycle_fd;
};


