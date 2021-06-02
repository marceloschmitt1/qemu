#ifndef QEMU_AD7150_H
#define QEMU_AD7150_H

#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_AD7150 "ad7150"
OBJECT_DECLARE_SIMPLE_TYPE(AD7150State, AD7150)

struct AD7150State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq *pin;

    uint8_t pointer;
    uint8_t config;
    int16_t temperature;
    int64_t cap[2];
    uint64_t cap_avg[2];
    uint64_t sensitivity_or_thresh_high[2];
    uint64_t timeout_or_thresh_low[2];
    bool pwrdwn;
    bool out[2]; 
};

#endif
