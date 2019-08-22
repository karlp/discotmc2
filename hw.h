/*
 * March 2017 Karl Palsson <karlp@tweak.net.au>
 */

#include <stdbool.h>
#include <stdint.h>

#pragma once

struct hw_detail
{
       uint32_t led_rcc;
       uint32_t led_port;
       uint32_t led_pin;
};


extern struct hw_detail hw_details;

#ifdef __cplusplus
extern "C" {
#endif

/**
        * Expected to setup clocks, turn on all peripherals, and configure
        * any gpios necessary.
        * @param hw pointer to hw details necessary
        */
       void hw_setup(struct hw_detail* hw);

       /* let devices have a status led */
       void hw_set_led(bool val);

#ifdef __cplusplus
}
#endif
