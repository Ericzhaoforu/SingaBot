/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>
#include <AP_Math/crc.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
uint8_t message[] = {0x04,0x0F,0x20,0x01,0x66,0x10,0x00,0x0A,0x9F,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0xCC};
/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that hal.serial(n) can be initialized

    setup_uart(hal.serial(0), "SERIAL0");  // console
    setup_uart(hal.serial(1), "SERIAL1");  // telemetry 1
    setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS
}

// static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
// {
//     if (uart == nullptr) {
//         // that UART doesn't exist on this platform
//         return;
//     }
//     uart->printf("Hello on UART %s at %.3f seconds\n",
//                  name, (double)(AP_HAL::millis() * 0.001f));
// }

void loop(void)
{
    // test_uart(hal.serial(0), "SERIAL0");
    // test_uart(hal.serial(1), "SERIAL1");
    // test_uart(hal.serial(2), "SERIAL2");
    // test_uart(hal.serial(3), "SERIAL3");
    // test_uart(hal.serial(4), "SERIAL4");
    hal.console->printf("bcc:%d\n",bcc_check(message,18-2));
    // also do a raw printf() on some platforms, which prints to the
    // debug console
    //::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
